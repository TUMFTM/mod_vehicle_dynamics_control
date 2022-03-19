import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
import json
import socket
import struct


class MPCVisualizer:
    def __init__(self, track_file):
        # initialize parameters
        self.param = {
            'pred_horizon_m': 100, # prediction horizon used for visualization in meter
            'visz_offset': 30 # offset added to visualization
        }

        # initialize UDP interface
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 5000))
        self.sock.setblocking(False)


        # load track
        self.track_file = track_file
        with open(self.track_file) as json_file:
            self.track = json.load(json_file)
            self.inside_bound = np.array(self.track["Inside"])
            self.outside_bound = np.array(self.track["Outside"])

        # initialize GUI
        self.fig = plt.figure()
        self.ax = plt.gca()
        plt.grid(self.ax)

        # visualize track
        self.plt_inside_bound = plt.plot(self.inside_bound[:, 0], self.inside_bound[:, 1], 'k')
        self.plt_outside_bound = plt.plot(self.outside_bound[:, 0], self.outside_bound[:, 1], 'k')

        self.ax.set_xlim(-350, 350)
        self.ax.set_ylim(-400, 200)

        self.ax.set_xlabel('East in m')
        self.ax.set_ylabel('North in m')

        # initialize vehicle data
        self.vehicle = {
            'x': 0,
            'y': 0,
            'psi': 0,
            'x_pred': [0]*50,
            'y_pred': [0]*50,
            'v_pred': [0]*50,
            'x_pred_left': [0]*50,
            'y_pred_left': [0]*50,
            'x_pred_right': [0]*50,
            'y_pred_right': [0]*50
        }

        # initialize prediction plots
        norm = plt.Normalize(10, 40)
        segment = [[(x, y) for x, y in zip(self.vehicle['x_pred_left'], self.vehicle['y_pred_left'])]]
        self.pred_left = LineCollection(segment, cmap='viridis', norm=norm)
        self.pred_left.set_array(np.array(self.vehicle['v_pred']))
        self.pred_left.set_linewidth(3)
        self.plt_pred_left = self.ax.add_collection(self.pred_left)
        segment = [[(x, y) for x, y in zip(self.vehicle['x_pred_right'], self.vehicle['y_pred_right'])]]
        self.pred_right = LineCollection(segment, cmap='viridis', norm=norm)
        self.pred_right.set_array(np.array(self.vehicle['v_pred']))
        self.pred_right.set_linewidth(3)
        self.plt_pred_right = self.ax.add_collection(self.pred_right)


        # load vehicle
        vehicle_im = plt.imread("Vehicle.png")
        self.vehicle_im = self.ax.imshow(vehicle_im, interpolation='none',
                   origin='lower',
                   extent=[-2.5, 2.5, -1, 1], clip_on=True)


    def check_for_data(self):
        data = None
        try:
            while True:
                data, _ = self.sock.recvfrom(1424)
        except socket.error:
            pass

        if data is not None:
            data_converted = struct.unpack('ddd350f', data)
            self.vehicle['x'] = data_converted[0]
            self.vehicle['y'] = data_converted[1]
            self.vehicle['psi'] = data_converted[2]
            self.vehicle['x_pred'] = data_converted[3:53]
            self.vehicle['y_pred'] = data_converted[53:103]
            self.vehicle['v_pred'] = data_converted[103:153]
            self.vehicle['x_pred_left'] = data_converted[153:203]
            self.vehicle['y_pred_left'] = data_converted[203:253]
            self.vehicle['x_pred_right'] = data_converted[253:303]
            self.vehicle['y_pred_right'] = data_converted[303:353]

    def update(self):
        # move vehicle to correct position
        rot_vehicle = mtransforms.Affine2D().rotate(self.vehicle['psi'] + np.pi/2)
        trans_vehicle = mtransforms.Affine2D().translate(self.vehicle['x'], self.vehicle['y'])
        final_trans = rot_vehicle + trans_vehicle + self.ax.transData
        self.vehicle_im.set_transform(final_trans)

        # update predictions
        self.plt_pred_left.set_segments([[(x, y) for x, y in zip(self.vehicle['x_pred_left'], self.vehicle['y_pred_left'])]])
        self.plt_pred_left.set_array(np.array(self.vehicle['v_pred']))
        self.plt_pred_right.set_segments([[(x, y) for x, y in zip(self.vehicle['x_pred_right'], self.vehicle['y_pred_right'])]])
        self.plt_pred_right.set_array(np.array(self.vehicle['v_pred']))

        # set nice axis limits
        future_pred_vehicle_x = self.vehicle['x'] - self.param['pred_horizon_m']*np.sin(self.vehicle['psi'])
        future_pred_vehicle_y = self.vehicle['y'] + self.param['pred_horizon_m']*np.cos(self.vehicle['psi'])
        # find maximum distance
        x_dist = abs(future_pred_vehicle_x - self.vehicle['x'])
        y_dist = abs(future_pred_vehicle_y - self.vehicle['y'])
        x_mean = (future_pred_vehicle_x + self.vehicle['x'])/2
        y_mean = (future_pred_vehicle_y + self.vehicle['y'])/2
        # correction factor reducing the oscillation dependend on the orientation
        corr_fac = (np.sqrt(2) - 1)*abs(np.sin(self.vehicle['psi'] - np.pi/4)) + 1
        visz_width = max(x_dist, y_dist)/(2*corr_fac) + self.param['visz_offset']
        self.ax.set_xlim(x_mean - visz_width, x_mean + visz_width)
        self.ax.set_ylim(y_mean - visz_width, y_mean + visz_width)

        plt.pause(0.02)


myVisz = MPCVisualizer('Modena.json')
while True:
    myVisz.check_for_data()
    myVisz.update()
