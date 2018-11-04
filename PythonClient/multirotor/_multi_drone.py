import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint

import msgpackrpc
import time
import base64


image_folder = 'D:\\airsim_v1.2.0\\screenshot'
image_id = {
    'Drone1': 1,
    'Drone2': 1,
}

def take_picture(drone_name, is_save=True):
    responses = client.simGetImages([
        airsim.ImageRequest("front_center", airsim.ImageType.Scene),
        # airsim.ImageRequest("bottom_center", airsim.ImageType.Scene),
    ], vehicle_name=drone_name)
    if is_save:
        drone_image_folder = '{}\\{}'.format(image_folder, drone_name)
        if not os.path.isdir(drone_image_folder):
            os.makedirs(drone_image_folder)
        for idx, response in enumerate(responses):
            if response.compress: #png format
                print('image type {}, size {}'.format(
                    response.image_type, len(response.image_data_uint8)))
                filename = '{}\\{}-{}.png'.format(
                    drone_image_folder, image_id[drone_name], idx)
                image_id[drone_name] += 1
                airsim.write_file(filename, response.image_data_uint8)
                print('save image: {}'.format(filename))
            else:
                print('error: image format not support')
    return responses

def send_image(image_data_uint8, server_ip, server_port):
    myrpcclient = msgpackrpc.Client(
        msgpackrpc.Address(server_ip, server_port))
    data = base64.b64encode(image_data_uint8)
    result = myrpcclient.call('push', data, time.time())
    print('image sent to {}:{}, result={}'.format(
        server_ip, server_port, result))
    return result

def get_cur_pos(vehicle_name=''):
    cur_state = client.getMultirotorState(vehicle_name=vehicle_name)
    return cur_state.kinematics_estimated.position


def move_drone(drone_name, dx, dy, dz, yaw, speed):
    cur_pos = get_cur_pos(vehicle_name=drone_name)
    next_pos = airsim.Vector3r(
        cur_pos.x_val + dx, cur_pos.y_val + dy, cur_pos.z_val + dz)
    print("try to move: {} -> {}, yaw={}, speed={}".format(
        cur_pos, next_pos, yaw, speed))
    rc = client.moveToPositionAsync(
        next_pos.x_val, next_pos.y_val, next_pos.z_val, speed,
        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw), 
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        vehicle_name=drone_name).join()
    cur_pos = get_cur_pos(vehicle_name=drone_name)
    print(cur_pos)

def move_to_pos(drone_name, x, y, z, yaw, speed):
    cur_pos = get_cur_pos(vehicle_name=drone_name)
    print("try to move: {} -> {}, yaw={}, speed={}".format(
        cur_pos, (x, y, z), yaw, speed))
    rc = client.moveToPositionAsync(
        x, y, z, speed,
        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw), 
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        vehicle_name=drone_name).join()
    cur_pos = get_cur_pos(vehicle_name=drone_name)
    print(cur_pos)

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")

f1 = client.takeoffAsync(vehicle_name="Drone1")
f2 = client.takeoffAsync(vehicle_name="Drone2")
f1.join()
f2.join()

state1 = client.getMultirotorState(vehicle_name="Drone1")
s = pprint.pformat(state1)
print("state: %s" % s)
state2 = client.getMultirotorState(vehicle_name="Drone2")
s = pprint.pformat(state2)
print("state: %s" % s)

airsim.wait_key('Press any key to move Drone1')

for i in range(0, 15):
    move_drone('Drone1', 3, 0, 0, 90, 1)
    responses = take_picture('Drone1')
    result = send_image(responses[0].image_data_uint8, '172.17.20.12', 18800)
    print("===> findObj: {}".format(result))
    if 'car' in result:
        break

airsim.wait_key('Press any key to move Drone2')

next_pos = get_cur_pos(vehicle_name='Drone1')
rc = move_to_pos('Drone2', next_pos.x_val, next_pos.y_val, next_pos.z_val-3, 0, 20)
responses = take_picture('Drone2')

airsim.wait_key('Press any key to reset to original state')
client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")
