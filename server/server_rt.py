import asyncio
import functools
import json
import math
import threading
import time
import uuid

import websockets as ws

from drone import Drone
from pilot import Pilot

cfg_data = {
    'ip': '0.0.0.0',
    'port': 5678,
    'sim_freq': 100,
    'ctrl_freq': 600
}


def loop(freq_hz, func_update):
    param_moment_interval = 1.0 / freq_hz
    param_calc_max = param_moment_interval * 0.9
    ts_moment_start = time.time()
    while True:
        ts_moment_past = time.time() - ts_moment_start
        if param_moment_interval > ts_moment_past:
            time.sleep(param_moment_interval - ts_moment_past)
        ts_calc_start = ts_moment_end = time.time()
        func_update(ts_moment_end - ts_moment_start)
        ts_calc_end = time.time()
        ts_moment_start = ts_moment_end
        ts_calc_cost = ts_calc_end - ts_calc_start
        if param_calc_max < ts_calc_cost:
            print('WARN: one update cost %.3fs@%.3f' % (ts_calc_cost, ts_calc_start))


class DroneUsr:
    def __init__(self, connection):
        self.conn = connection
        self.uid = str(uuid.uuid1())
        self.drone = Drone()
        self.loop_drone = threading.Thread(target=loop, args=(cfg_data['sim_freq'], self.drone.update))
        self.loop_drone.start()
        self.pilot = Pilot(self.drone)
        self.loop_pilot = threading.Thread(target=loop, args=(cfg_data['ctrl_freq'], self.pilot.update))
        self.loop_pilot.start()
        self.pilot.set_target((0.25, 0.25, -0.25), math.pi / 180 * 45)
        # self.pilot.ctrl_attitude.set_target(0, 0, math.pi / 180 * 45)

    @property
    def id(self):
        return self.uid

    @property
    def ip(self):
        return self.conn.remote_address[0]

    @property
    def model(self):
        return self.drone

    async def send(self, message):
        await self.conn.send(json.dumps(message))

    async def update(self):
        msg = {
            'cmd': 'RotationMatrix_SC',
            'm': self.drone.matrix4
        }
        await self.conn.send(json.dumps(msg))

    def __repr__(self):
        return '%s (%s)' % (self.uid, self.ip)


class DroneSrv:

    def __init__(self):
        self.m_users = {}

    def add(self, conn):
        user = DroneUsr(conn)
        self.m_users[user.id] = user
        print('%s ~ %d users connected' % (user, len(self.m_users)))
        return user

    def remove(self, user_id):
        self.m_users.pop(user_id)
        print('%d users connected' % len(self.m_users))

    async def process(self, user_id, message):
        data = json.loads(message)
        if 'cmd' in data:
            command = data['cmd']
            if 'RotationMatrix_CS' == command:
                await self.on_rm_requested(user_id)
            elif 'Operation_CS' == command:
                op_code = data['op']
                await self.on_op_requested(user_id, op_code)
            else:
                print('invalid command')

    async def on_rm_requested(self, user_uid):
        user = self.m_users[user_uid]
        notify_msg = {
            'cmd': 'RotationMatrix_SC',
            'm': user.model.matrix4
        }
        await user.send(notify_msg)

    async def on_op_requested(self, user_uid, op_code):
        user = self.m_users[user_uid]
        user.model.on_operation(op_code)
        notify_msg = {
            'cmd': 'RotationMatrix_SC',
            'm': user.model.matrix4
        }
        await user.send(notify_msg)


async def ws_handler(conn, path, server):
    # print(path)
    client = None
    try:
        client = server.add(conn)
        if client is not None:
            async for message in conn:
                await server.process(client.id, message)
    except ws.ConnectionClosedError as ex:
        print(ex)
        # code = 1005 (no status code [internal]), no reason
    finally:
        print('handler quit')
        if client is not None:
            server.remove(client.id)


async def main():
    srv = DroneSrv()
    bound_handler = functools.partial(ws_handler, server=srv)
    async with ws.serve(bound_handler, cfg_data['ip'], cfg_data['port']):
        await asyncio.Future()  # run forever


if __name__ == '__main__':
    asyncio.run(main())
