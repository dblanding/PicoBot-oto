import asyncio
import json
from pprint import pprint
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from geom2d import pt_coords
from robot_ble_connection import BleConnection

# Offset between sensor value and actual distance to robot center
R_OFFSET = -14
L_OFFSET = -5
F_OFFSET = 33

class RobotDisplay:
    def __init__(self):
        self.ble_connection = BleConnection(self.handle_data)
        self.buffer = ""
        self.arena = {}
        self.closed = False
        self.fig, self.axes = plt.subplots()
        self.pose_list = []
        self.poses = None
        self.r_pnts_list = []
        self.r_pnts = None
        self.l_pnts_list = []
        self.l_pnts = None
        self.f_pnts_list = []
        self.f_pnts = None

    def handle_close(self, _):
        self.closed = True

    def handle_data(self, data):
        self.buffer += data.decode()
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            # print(f"Received data: {line}")
            try:
                message = json.loads(line)
                pprint(message)
            except ValueError:
                print("Error parsing JSON")
                return
            if "arena" in message:
                self.arena = message
            if "pose" in message:
                pose = message["pose"]
                self.pose_list.append(pose)
                self.poses = np.array(self.pose_list, dtype=np.float32)
            if "dist_R" in message:
                r_dist = message["dist_R"] + R_OFFSET
                if r_dist < 1500:
                    r_point = pt_coords(pose, r_dist/1000, 'R')
                    self.r_pnts_list.append(r_point)
                    self.r_pnts = np.array(self.r_pnts_list, dtype=np.float32)
            if "dist_L" in message:
                l_dist = message["dist_L"] + L_OFFSET
                if l_dist < 1500:
                    l_point = pt_coords(pose, l_dist/1000, 'L')
                    self.l_pnts_list.append(l_point)
                    self.l_pnts = np.array(self.l_pnts_list, dtype=np.float32)
            if "dist_F" in message:
                f_dist = message["dist_F"] + F_OFFSET
                if f_dist < 1500:
                    f_point = pt_coords(pose, f_dist/1000, 'F')
                    self.f_pnts_list.append(f_point)
                    self.f_pnts = np.array(self.f_pnts_list, dtype=np.float32)

    def draw(self):
        self.axes.clear()
        if self.arena:
            for line in self.arena["arena"]:
                self.axes.plot(
                    [line[0][0], line[1][0]], [line[0][1], line[1][1]], color="black"
                )
        if self.poses is not None:
            self.axes.scatter(self.poses[:,0], self.poses[:,1], color="blue")
        if self.r_pnts is not None:
            self.axes.scatter(self.r_pnts[:,0], self.r_pnts[:,1], color="green")
        if self.l_pnts is not None:
            self.axes.scatter(self.l_pnts[:,0], self.l_pnts[:,1], color="red")
        if self.f_pnts is not None:
            self.axes.scatter(self.f_pnts[:,0], self.f_pnts[:,1], color="yellow")
        

    async def send_command(self, command):
        request = (json.dumps({"command": command})  ).encode()
        print(f"Sending request: {request}")
        await self.ble_connection.send_uart_data(request)

    def tele(self, _):
        self.button_task = asyncio.create_task(self.send_command("tele"))

    def auto(self, _):
        self.button_task = asyncio.create_task(self.send_command("auto"))

    def start(self, _):
        self.button_task = asyncio.create_task(self.send_command("start"))

    def stop(self, _):
        self.button_task = asyncio.create_task(self.send_command("stop"))

    async def main(self):
        plt.ion()
        await self.ble_connection.connect()
        try:
            await self.send_command("arena")
            self.fig.canvas.mpl_connect("close_event", self.handle_close)
            tele_button = Button(plt.axes([0.2, 0.85, 0.1, 0.075]), "Tele")
            tele_button.on_clicked(self.tele)
            auto_button = Button(plt.axes([0.4, 0.85, 0.1, 0.075]), "Auto")
            auto_button.on_clicked(self.auto)
            start_button = Button(plt.axes([0.6, 0.85, 0.1, 0.075]), "Start")
            start_button.on_clicked(self.start)
            stop_button = Button(plt.axes([0.8, 0.85, 0.1, 0.075]), "Stop")
            stop_button.on_clicked(self.stop)
            while not self.closed:
                self.draw()
                plt.draw()
                plt.pause(0.05)
                await asyncio.sleep(0.01)
        finally:
            await self.ble_connection.close()


robot_display = RobotDisplay()
asyncio.run(robot_display.main())
