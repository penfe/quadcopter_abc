from math import sin, cos, tan, pi
import numpy as np


def get_control(d, ct, cm, motors):
    (w1, w2, w3, w4) = motors
    (ww1, ww2, ww3, ww4) = (w1 ** 2, w2 ** 2, w3 ** 2, w4 ** 2)
    force = ct * (ww1 + ww2 + ww3 + ww4)
    tau_x = d * ct * np.sqrt(2) / 2 * (ww1 - ww2 - ww3 + ww4)
    tau_y = d * ct * np.sqrt(2) / 2 * (ww1 + ww2 - ww3 - ww4)
    tau_z = cm * (ww1 - ww2 + ww3 - ww4)
    return force, tau_x, tau_y, tau_z


def dynamic_position(f, m, g, phi, theta, psi):
    x_dot = -f / m * (cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))
    y_dot = -f / m * (sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))
    z_dot = g - f / m * (cos(phi)*cos(theta))
    return x_dot, y_dot, z_dot


def dynamic_attitude(tau_arr, motors, inertia_arr, j_rp, p, q, r):
    (tau_x, tau_y, tau_z) = tau_arr
    (w1, w2, w3, w4) = motors
    (inertia_xx, inertia_yy, inertia_zz) = inertia_arr
    omega = -w1 + w2 - w3 + w4
    p_dot = (1 / inertia_xx) * (tau_x + q * r * (inertia_yy - inertia_zz) - j_rp * q * omega)
    q_dot = (1 / inertia_yy) * (tau_y + p * r * (inertia_zz - inertia_xx) + j_rp * p * omega)
    r_dot = (1 / inertia_zz) * (tau_z + p * q * (inertia_xx - inertia_yy))
    return p_dot, q_dot, r_dot


def transform(phi, theta, psi, p, q, r):
    w = np.array([
        [1, tan(theta)*sin(phi), tan(theta)*cos(phi)],
        [0, cos(phi), -sin(phi)],
        [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]])
    w_body = np.array([p, q, r])
    w_earth = np.matmul(w, w_body.T)
    return w_earth[0], w_earth[1], w_earth[2]


class Drone:
    def __init__(self):
        self.centre = np.zeros(3)  # 重心位置
        self.attitude = np.zeros(3)  # 姿态欧拉角
        self.v = np.zeros(3)  # 线速度
        self.w = np.zeros(3)  # 角速度
        self.motors = np.zeros(4)  # 四个螺旋桨转速
        self.g = 9.8  # 地球重力加速度
        self.d_m = 0.225  # 螺旋桨到质心距离
        self.m_kg = 1.4  # 总质量
        self.ct = 1.105e-05  # 拉力系数
        self.cm = 1.779e-07*2  # 力矩系数
        self.inertia_xx = 0.0211  # X 轴转动惯量
        self.inertia_yy = 0.0219  # Y 轴转动惯量
        self.inertia_zz = 0.0366  # Z 轴转动惯量
        self.j_rp = 0.0001287  # 总转动惯量 kg*m^2
        w_init = np.sqrt(self.m_kg * self.g / (4 * self.ct))
        self.motors = np.array([w_init * 1, w_init * 1, w_init * 1, w_init * 1])
        # self.motors = np.array([w_init * 1.01, w_init * 0.99, w_init * 0.99, w_init * 1.01])  # ROLL
        # self.motors = np.array([w_init * 0.99, w_init * 0.99, w_init * 1.01, w_init * 1.01])  # PITCH
        # self.motors = np.array([w_init * 1.01, w_init * 0.99, w_init * 1.01, w_init * 0.99])  # YAW

    @property
    def rm(self):
        (phi, theta, psi) = self.attitude
        return np.array([
            [cos(theta)*cos(psi), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)],
            [cos(theta)*sin(psi), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi)],
            [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)]])

    @property
    def matrix4(self):
        m = np.identity(4)
        m[0:3, 0:3] = self.rm
        m[:-1, -1] = self.centre
        return m.flatten(order='F').tolist()

    @property
    def ctrl_matrix(self):
        elem = (np.sqrt(2) / 2) * self.d_m * self.ct
        mtx = np.array([
            [self.ct, self.ct, self.ct, self.ct],
            [elem, -elem, -elem, elem],
            [elem, elem, -elem, -elem],
            [self.cm, -self.cm, self.cm, -self.cm]
        ])
        return mtx

    def update(self, secs):
        force, tau_x, tau_y, tau_z = get_control(self.d_m, self.ct, self.cm, self.motors)
        (phi, theta, psi) = self.attitude
        x_dot, y_dot, z_dot = dynamic_position(force, self.m_kg, self.g, phi, theta, psi)
        (vdx, vdy, vdz) = (x_dot * secs, y_dot * secs, z_dot * secs)
        v_eps = 1e-12
        self.v[0] += vdx if v_eps < abs(vdx) else 0
        self.v[1] += vdy if v_eps < abs(vdy) else 0
        self.v[2] += vdz if v_eps < abs(vdz) else 0
        self.centre += self.v * secs
        tau_arr = (tau_x, tau_y, tau_z)
        inertia_arr = (self.inertia_xx, self.inertia_yy, self.inertia_zz)
        (p, q, r) = self.w
        p_dot, q_dot, r_dot = dynamic_attitude(tau_arr, self.motors, inertia_arr, self.j_rp, p, q, r)
        # p_dot, q_dot, r_dot = transform(phi, theta, psi, p_dot, q_dot, r_dot)
        (wdx, wdy, wdz) = (p_dot * secs, q_dot * secs, r_dot * secs)
        w_eps = 1e-12
        self.w[0] += wdx if w_eps < abs(wdx) else 0
        self.w[1] += wdy if w_eps < abs(wdy) else 0
        self.w[2] += wdz if w_eps < abs(wdz) else 0
        self.attitude += self.w * secs
        # self.attitude %= 2 * pi

    def set_motors(self, motors):
        self.motors = motors

    def on_operation(self, op_code):
        print(op_code)
        if 'L' == op_code:
            self.rm = np.array([
                [0, -1, 0],
                [0, 0, -1],
                [1, 0, 0]])
        elif 'R' == op_code:
            self.rm = np.array([
                [1, 0, 0],
                [0, 0, -1],
                [0, 1, 0]])
        elif 'T' == op_code:
            self.rm = np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
        elif 'B' == op_code:
            self.rm = np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
        else:
            self.rm = np.identity(3)
        return self.matrix4
