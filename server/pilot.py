import math
from math import cos, sin, pi
import numpy as np
import sys

from drone import Drone


class PositionControl:
    def __init__(self):
        self.target = np.zeros(3)
        self.p2v = np.array([1e-01, 1e-01, 1e-01])
        self.err_v = None  # 上次速度偏差
        self.err_vi = np.zeros(3)  # 速度偏差累积
        self.err_vi_max = np.array([1e-02, 1e-02, 1e-01])
        self.K_p = np.diag([3 / 4, 3 / 4, 4 / 5])
        self.K_i = np.diag([1 / 32, 1 / 32, 1 / 32])
        self.K_d = np.diag([-2 / 3, -2 / 3, -2 / 3])
        self.v_max = np.array([1e+00, 1e+00, 1e+00])  # 速度限幅
        self.a_max = np.array([1e-01, 1e-01, 1e-01])  # 加速度限幅

    def set_target(self, target):
        self.target = target
        self.err_v = None
        self.err_vi = np.zeros(3)

    def update(self, measured):
        (p_m, v_m) = measured
        v_d = (self.target - p_m) * self.p2v
        for i in range(len(v_d)):
            if abs(v_d[i]) > self.v_max[i]:
                v_d[i] /= abs(v_d[i]) / self.v_max[i]
        err_v = v_d - v_m
        err_v_dot = np.zeros(3) if self.err_v is None else err_v - self.err_v
        self.err_v = err_v
        self.err_vi += err_v
        err_vi_limited = self.err_vi.copy()
        for i in range(len(err_vi_limited)):
            if abs(err_vi_limited[i]) > self.err_vi_max[i]:
                err_vi_limited[i] /= abs(err_vi_limited[i]) / self.err_vi_max[i]
        v_dot = np.matmul(self.K_p, err_v) + np.matmul(self.K_i, err_vi_limited) + np.matmul(self.K_d, err_v_dot)
        # print(p_m[0], v_m[0], v_d[0], v_dot[0], err_v[0], self.err_vi[0], err_v_dot[0])
        for i in range(len(v_dot)):
            if abs(v_dot[i]) > self.a_max[i]:
                v_dot[i] /= abs(v_dot[i]) / self.a_max[i]
        return v_dot

    @staticmethod
    def solve(v_dot, g, psi, m):
        vh_dot = np.array([v_dot[:-1]]).T
        mtx_psi = np.array([
            [cos(psi), sin(psi)],
            [sin(psi), -cos(psi)]
        ])
        # att_h = np.matmul(np.linalg.pinv(mtx_psi), vh_dot / -g)
        att_h = np.matmul(mtx_psi.T, vh_dot) / -g
        (theta_d, phi_d) = att_h[:, 0]
        vz_dot = v_dot[2]
        f_d = m * (g - vz_dot)
        return f_d, phi_d, theta_d


class AttitudeControl:
    def __init__(self):
        self.phi_d = 0
        self.theta_d = 0
        self.psi_d = 0
        self.err_w = None  # 上次角速度偏差
        self.err_wi = np.zeros((3, 1))  # 角速度偏差累积
        self.err_wi_max = np.array([pi / 2, pi / 2, pi / 2])
        self.K_p = np.diag([2 / 3, 2 / 3, 2 / 3])
        self.K_i = np.diag([1.0 / 32, 1.0 / 32, 1.0 / 32])
        self.K_d = np.diag([-1.0 / 4, -1.0 / 4, -1.0 / 4])
        self.w_max = np.array([[pi / 180 * 30, pi / 180 * 30, pi / 180 * 30]]).T  # 角速度限幅
        self.w_dot_max = np.array([[pi / 180 * 3, pi / 180 * 3, pi / 180 * 3]]).T  # 角加速度限幅

    def set_target(self, phi_d, theta_d, psi_d):
        self.phi_d = phi_d
        self.theta_d = theta_d
        self.psi_d = psi_d
        self.err_w = None
        self.err_wi = np.zeros((3, 1))

    def update(self, attitude, w):
        (phi_c, theta_c, psi_c) = attitude
        (p_c, q_c, r_c) = w
        w_d = np.array([[self.phi_d - phi_c, self.theta_d - theta_c, self.psi_d - psi_c]]).T
        for i in range(len(w_d)):
            if abs(w_d[i]) > self.w_max[i]:
                w_d[i] /= abs(w_d[i]) / self.w_max[i]
        w_c = np.array([[p_c, q_c, r_c]]).T
        err_w = w_d - w_c
        self.err_wi += err_w
        err_wi_limited = self.err_wi.copy()
        for i in range(len(err_wi_limited)):
            if abs(err_wi_limited[i]) > self.err_wi_max[i]:
                err_wi_limited[i] /= abs(err_wi_limited[i]) / self.err_wi_max[i]
        err_w_dot = np.zeros((3, 1)) if self.err_w is None else err_w - self.err_w
        self.err_w = err_w
        w_dot = np.matmul(self.K_p, err_w) + np.matmul(self.K_i, err_wi_limited) + np.matmul(self.K_d, err_w_dot)
        for i in range(len(w_dot)):
            if abs(w_dot[i]) > self.w_dot_max[i]:
                w_dot[i] /= abs(w_dot[i]) / self.w_dot_max[i]
        return w_dot[:, 0]

    @staticmethod
    def solve_tau(w_dot, j=1.0):
        return j * w_dot


class Pilot:
    def __init__(self, drone):
        self.drone = drone
        self.psi_d = 0
        self.ctrl_position = PositionControl()
        self.ctrl_attitude = AttitudeControl()
        self.ctrl_mtx_inv = np.linalg.pinv(drone.ctrl_matrix)

    def set_target(self, pos_d, psi_d):
        self.ctrl_position.set_target(np.array(pos_d))
        self.psi_d = psi_d

    def update(self, secs):
        v_dot = self.ctrl_position.update((self.drone.centre, self.drone.v))
        f_d, phi_d, theta_d = self.ctrl_position.solve(v_dot, self.drone.g, self.drone.attitude[2], self.drone.m_kg)
        self.ctrl_attitude.set_target(phi_d, theta_d, self.psi_d)
        # #
        # v_dot = (math.nan, math.nan, math.nan)
        # (f_d, phi_d, theta_d) = (13.720000, self.ctrl_attitude.phi_d, self.ctrl_attitude.theta_d)
        # #
        w_dot = self.ctrl_attitude.update(self.drone.attitude, self.drone.w)
        # print(self.drone.v, v_dot, (f_d, phi_d, theta_d), w_dot)
        tau_d = self.ctrl_attitude.solve_tau(w_dot)
        motors = self.__update_motors_ctrl(f_d, tau_d[0], tau_d[1], tau_d[2], self.ctrl_mtx_inv)
        self.drone.set_motors(motors)
        return v_dot, w_dot, f_d, phi_d, theta_d, self.psi_d, tau_d

    @staticmethod
    def __update_motors_ctrl(force, tau_x, tau_y, tau_z, mtx_inv):
        desire = np.array([[force, tau_x, tau_y, tau_z]]).T
        motors = np.sqrt(np.matmul(mtx_inv, desire))
        return motors[:, 0]


def main(argv=None):
    print(argv)
    drone = Drone()
    pilot = Pilot(drone)
    pilot.set_target((0.25, 0.25, 0.25), math.pi / 180 * 45)
    # pilot.ctrl_attitude.set_target(0, 0, math.pi / 180 * 45)
    interval = 1 / 1000
    (cnt, secs) = (0, 60)
    with open('__pilot_log.csv', 'w') as f:
        head = 'px,py,pz,vx,vy,vz,roll,pitch,yaw,wx,wy,wz,vdx,vdy,vdz,wdx,wdy,wdz,f,phi,theta,psi,tx,ty,tz'
        print(head)
        f.write('%s\n' % head)
        while True:
            drone.update(interval)
            v_dot, w_dot, f_d, phi_d, theta_d, psi_d, tau_d = pilot.update(interval)
            info = '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f' % (
                drone.centre[0], drone.centre[1], drone.centre[2],
                drone.v[0], drone.v[1], drone.v[2],
                drone.attitude[0], drone.attitude[1], drone.attitude[2],
                drone.w[0], drone.w[1], drone.w[2],
                v_dot[0], v_dot[1], v_dot[2],
                w_dot[0], w_dot[1], w_dot[2],
                f_d, phi_d, theta_d, psi_d,
                tau_d[0], tau_d[1], tau_d[2]
            )
            print(info)
            f.write('%s\n' % info)
            cnt += interval
            if secs < cnt:
                break


if __name__ == '__main__':
    sys.exit(main(sys.argv))
