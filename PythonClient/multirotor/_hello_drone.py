"""
For connecting to the AirSim drone environment and testing API functionality
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint

import sys
import math
import logging
import msgpackrpc
from PIL import Image
import time


logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger()


class mininetHelper:
    _SERVER_ADDR = '192.168.56.102:19000'
    _DRONE_ADDR = '192.168.56.102:18000'

    def __init__(self, server_addr=None, drone_addr=None):
        if not server_addr:
            server_addr = self._SERVER_ADDR
        if not drone_addr:
            drone_addr = self._DRONE_ADDR
        ip, port = server_addr.split(':')
        self.server_ip = ip
        self.server_port = int(port)
        ip, port = drone_addr.split(':')
        self.drone_ip = ip
        self.drone_port = int(port)
    
    def send_image(self, filename):
        with open(filename, 'r') as fd:
            myrpcclient = msgpackrpc.Client(
                msgpackrpc.Address(self.drone_ip, self.drone_port))
            result = myrpcclient.call(
                'send', time.time(), response.image_data_uint8)
            log.info('image sent to forwarder: {}'.format(result))

    def move_drone(self, origin_pos, next_pos, speed):
        log.info("move drone in mininet")
        rel_x, rel_y, rel_z = (
            origin_pos.x_val - next_pos.x_val, 
            origin_pos.y_val - next_pos.y_val, 
            origin_pos.z_val - next_pos.z_val)
        myrpcclient = msgpackrpc.Client(
            msgpackrpc.Address(self.server_ip, self.server_port))
        myrpcclient.call('move', time.time(), rel_x, rel_y, rel_z, speed)

class screenshotHelper:
    _DEFAULT_FOLDER = 'D:\\airsim_v1.2.0\\screenshot\\'

    def __init__(self, drone_name, client, image_folder=''):
        self.image_id = 1
        self.drone_name = drone_name
        self.client = client
        if not image_folder:
            image_folder = self._DEFAULT_FOLDER
        image_folder += '{}\\'.format(self.drone_name)
        if not os.path.isdir(image_folder):
            try:
                os.makedirs(image_folder)
            except OSError:
                log.error('cannot create dir: {}'.format(image_folder))
        self.image_folder = image_folder
        log.info('save image to: {}'.format(self.image_folder))

    def do_screenshot(self, is_display=False):
        """ get camera images from the car
        Returns:
            image_paths (list): filepaths of the screenshots
        """
        responses = self.client.simGetImages([
            airsim.ImageRequest("front_center", airsim.ImageType.Scene),
            airsim.ImageRequest("back_center", airsim.ImageType.Scene),
        ])
        log.debug('Retrieved images: %d' % len(responses))
        filename_prefix = self.image_folder + "{}-".format(self.image_id)
        self.image_id += 1
        image_paths = []
        for idx, response in enumerate(responses):
            filename = filename_prefix + str(idx)
            if response.compress: #png format
                log.debug('image type {}, size {}'.format(
                    response.image_type, len(response.image_data_uint8)))
                filename = os.path.normpath(filename + '.png')
                airsim.write_file(filename, response.image_data_uint8)
            else:
                log.error('error: image format not support')
            image_paths.append(filename)
        if is_display:
            self.display(image_paths)
        return image_paths

    @staticmethod
    def display(filepath_list):
        images = [Image.open(file) for file in filepath_list]
        padding = 5
        widths, heights = zip(*(i.size for i in images))
        total_width = sum(widths) + padding * len(images)
        max_height = max(heights)
        new_im = Image.new('RGB', (total_width, max_height))
        x_offset = 0
        for im in images:
            new_im.paste(im, (x_offset, 0))
            x_offset += im.size[0] + padding
        new_im.show()


class GpsUtils:
    _EARTH_RADIUS = 6378137.0

    @staticmethod
    def degreesToRadians(degrees):
        return math.pi * degrees / 180.0

    @classmethod
    def geodeticToNedFast(cls, geo, home):
        d_lat = geo.latitude - home.latitude;
        d_lon = geo.longitude - home.longitude;
        d_alt = home.altitude - geo.altitude;
        x = cls.degreesToRadians(d_lat) * cls._EARTH_RADIUS
        y = cls.degreesToRadians(d_lon) * (
            cls._EARTH_RADIUS * math.cos(cls.degreesToRadians(geo.latitude)))
        return airsim.Vector3r(x, y, d_alt)

    @staticmethod
    def getCalibratedPos(pos, ref):
        return airsim.Vector3r(
            pos.x_val - ref.x_val, pos.y_val - ref.y_val, pos.z_val - ref.z_val)

    @classmethod
    def getCalibratedPosFromGps(cls, gps, home, ref):
        pos = cls.geodeticToNedFast(gps, home)
        return airsim.Vector3r(
            pos.x_val - ref.x_val, pos.y_val - ref.y_val, pos.z_val - ref.z_val)


class DroneController:
    _DIRECTION_DICT = {
        # (move_x, move_y, move_z, yaw_diff)
        'A': (0, -1, 0, 0),
        'D': (0, +1, 0, 0),
        'W': (+1, 0, 0, 0),
        'S': (-1, 0, 0, 0),
        'R': (0, 0, -1, 0),
        'F': (0, 0, +1, 0),
        'Q': (+1, 0, 0, -45),  # W + yaw(angle)
        'E': (+1, 0, 0, +45),  # W + yaw(angle)
    }

    def __init__(self, drone_name, is_mininet_enabled=False):
        self.drone_name = drone_name
        self.is_mininet_enabled = is_mininet_enabled
        self.cur_yaw = 0
        self.scale = 1
        self.speed = 1

    @staticmethod
    def _compute_abs_direction(cur_yaw, scale, move_xyz_yaw):
        """
        Returns: dx, dy, dz
        """
        move_x, move_y, move_z, yaw_diff = move_xyz_yaw
        cur_yaw += yaw_diff
        if move_x != 0:
            move_x *= scale
            dx = move_x * math.cos(GpsUtils.degreesToRadians(cur_yaw))
            dy = move_x * math.sin(GpsUtils.degreesToRadians(cur_yaw))
            dz = move_z
        elif move_y != 0:
            move_y *= scale
            dx = move_y * math.cos(GpsUtils.degreesToRadians(cur_yaw + 90))
            dy = move_y * math.sin(GpsUtils.degreesToRadians(cur_yaw + 90))
            dz = move_z
        elif move_z != 0:
            move_z *= scale
            dx = move_x
            dy = move_y
            dz = move_z
        log.info("rel vector ({},{},{}) => abs vector ({},{}, {})".format(
            move_x, move_y, move_z, dx, dy, dz))
        return dx, dy, dz, cur_yaw

    @staticmethod
    def _init_drone_controller(drone_name):
        client = airsim.MultirotorClient()
        client.confirmConnection()
        client.enableApiControl(True, drone_name)
        client.armDisarm(True, drone_name)
        multirotor_state = client.getMultirotorState(vehicle_name=drone_name)
        log.info('multirotor_state={}'.format(multirotor_state))

        landed = multirotor_state.landed_state
        if landed == airsim.LandedState.Landed:
            log.info("taking off...")
            client.takeoffAsync(vehicle_name=drone_name).join()
        else:
            log.info("already flying...")
            client.hoverAsync(vehicle_name=drone_name).join()

        multirotor_state = client.getMultirotorState(vehicle_name=drone_name)
        home_gps_location = multirotor_state.gps_location
        pos = GpsUtils.geodeticToNedFast(home_gps_location, home_gps_location)
        log.info('gps={}, pos={}'.format(home_gps_location, pos))

        kinematics_state = client.simGetGroundTruthKinematics(
            vehicle_name=drone_name)
        log.debug('kinematics_state={}'.format(kinematics_state))
        return client

    @classmethod
    def _start_keyboard_controller(
            cls, client, drone_name, cur_yaw, scale, speed, is_mininet_enabled):
        # create helpers
        screenshot_helper = screenshotHelper(drone_name, client)
        mininet_helper = mininetHelper() if is_mininet_enabled else None

        display_info = (
            'Press AWSD,R(up)F(down)QE(turn left/right) key to move the drone.\n' 
            'Press P to take images.\n'
            'Press B key to reset to original state.\n'
            'Press O key to release API control.\n'
            'drone will be moved {}m with speed {}m/s.\n').format(scale, speed)

        while True:
            pressed_key = airsim.wait_key(display_info).decode('utf-8').upper()
            log.info("pressed_key={}".format(pressed_key))
            if pressed_key == 'P':
                image_file_list = screenshot_helper.do_screenshot(is_display=True)
                if mininet_helper is not None:
                    mininet_helper.send_image(image_file_list[0])
            elif pressed_key == 'O':
                client.enableApiControl(False, drone_name)
                break
            elif pressed_key == 'B':
                client.reset()
            else:
                move_xyz_yaw = cls._DIRECTION_DICT.get(pressed_key, None)
                if move_xyz_yaw is None:
                    log.error('invalid key')
                    continue
                # connect to the AirSim simulator        
                dx, dy, dz, cur_yaw = cls._compute_abs_direction(
                    cur_yaw, scale, move_xyz_yaw)
                result = cls._move_drone(
                    client, drone_name, dx, dy, dz, cur_yaw, speed,
                    mininet_helper=mininet_helper)
                assert result == True

    @classmethod
    def _move_drone(
            cls, client, drone_name, dx, dy, dz, cur_yaw, speed, 
            screenshot_helper=None, mininet_helper=None):
        # move the drong
        cur_state = client.getMultirotorState(vehicle_name=drone_name)
        cur_pos = cur_state.kinematics_estimated.position
        next_pos = airsim.Vector3r(
            cur_pos.x_val + dx, cur_pos.y_val + dy, cur_pos.z_val + dz)
        log.info("try to move: {} -> {}, yaw={}, speed={}".format(
            cur_pos, next_pos, cur_yaw, speed))
        rc = client.moveToPositionAsync(
            next_pos.x_val, next_pos.y_val, next_pos.z_val, speed,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=cur_yaw), 
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            vehicle_name=drone_name).join()
        # pose = client.simGetObjectPose(drone_name)
        # cur_pos = pose.position
        # next_pos = airsim.Vector3r(
        #     cur_pos.x_val + dx, cur_pos.y_val + dy, cur_pos.z_val + dz)
        # pose.position = next_pos
        # rc = client.simSetObjectPose(drone_name, pose, True)
        log.info("rc: {}".format(rc))
        if mininet_helper is not None:
            mininet_helper.move_drone(cur_pos, next_pos)
        # is collision?
        collision_info = client.simGetCollisionInfo(vehicle_name=drone_name)
        if collision_info.has_collided:
            log.error('collided! collision_info={}'.format(collision_info))
            client.enableApiControl(False, drone_name)
            return False
        if screenshot_helper is not None:
            image_file_list = screenshot_helper.do_screenshot(is_display=False)
            if mininet_helper is not None:
                mininet_helper.send_image(image_file_list[0])
        return True

    def start_keyboard_controller(self):
        client = self._init_drone_controller(self.drone_name)
        self._start_keyboard_controller(
            client, self.drone_name, self.cur_yaw, self.scale, self.speed, 
            self.is_mininet_enabled)

    def start_programmed_controller_case1(self):
        client = self._init_drone_controller(self.drone_name)
        screenshot_helper = screenshotHelper(self.drone_name, client)
        for i in range(0, 5):
            dx, dy, dz, cur_yaw, speed = 0, -6, -0.5, 0, 6
            self._move_drone(client, self.drone_name, 
                dx, dy, dz, cur_yaw, speed,
                screenshot_helper=screenshot_helper, 
                mininet_helper=None)
        log.info('end')

if __name__ == '__main__':
    usage = (
        'usage: python _hello_drone.py '
        '{ keyboard {Drone1 | Drone2  | ...} | programmed {case1} }')
    argc = len(sys.argv)
    if argc != 3:
        log.error(usage)
        exit(0)
    mode = sys.argv[1]
    if mode == 'keyboard':
        drone_name = sys.argv[2]
        log.info('start keyboard controller for {}'.format(drone_name))
        dc = DroneController(drone_name, is_mininet_enabled=False)
        dc.start_keyboard_controller()
    elif mode == 'programmed':
        case_name = sys.argv[2]
        log.info('start programmed controller for {}'.format(case_name))
        if case_name == 'case1':
            dc = DroneController('Drone1', is_mininet_enabled=False)
            dc.start_programmed_controller_case1()
        else:
            log.error('no such case: {}'.format(case_name))
    else:
        log.error('no such mode: {}'.format(mode))

