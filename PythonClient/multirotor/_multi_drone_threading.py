import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint

import msgpackrpc
import time
import base64
import threading
from msgpackrpc.error import RPCError


image_folder = 'C:\\NESLProjects\\airsim_v1.2.0\\screenshot'
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

def send_image_async(image_data_uint8, server_ip, server_port):
    myrpcclient = msgpackrpc.Client(
        msgpackrpc.Address(server_ip, server_port), timeout=1)
    data = base64.b64encode(image_data_uint8)
    ret = myrpcclient.call('push', data, time.time())
    print('image sent to {}:{}'.format(server_ip, server_port))
    return ret

def get_cur_pos(vehicle_name=''):
    cur_state = client.getMultirotorState(vehicle_name=vehicle_name)
    return cur_state.kinematics_estimated.position


def move_drone(drone_name, dx, dy, dz, yaw, speed, is_async=False):
    cur_pos = get_cur_pos(vehicle_name=drone_name)
    next_pos = airsim.Vector3r(
        cur_pos.x_val + dx, cur_pos.y_val + dy, cur_pos.z_val + dz)
    print("try to move: {} -> {}, yaw={}, speed={}".format(
        cur_pos, next_pos, yaw, speed))
    thread = client.moveToPositionAsync(
        next_pos.x_val, next_pos.y_val, next_pos.z_val, speed,
        yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw), 
        drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
        vehicle_name=drone_name)
    if is_async:
        return thread
    thread.join()
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


airsim.wait_key('Press any key to start workers')
is_car_found = threading.Event()
is_follow = threading.Event()
is_stop = threading.Event()

class rpcServer(object):
    def push(self, result, timestamp):
        print("recv result={}; sender_time={}".format(result, timestamp))
        if 'car' in result:
            is_car_found.set()
            is_follow.set()

    def stop(self, result, timestamp):
        print('stop simulation')
        is_stop.set()

server = msgpackrpc.Server(rpcServer())


def actuator(server):
    server.listen(msgpackrpc.Address("172.17.15.21", 18800))
    server.start()
    server.close()

def control_drone1_move(is_stop):
    while 1:
        while 1:
            try:
                thread = move_drone('Drone1', 45, 0, 0, 90, 5, is_async=True)
                time.sleep(15)
                thread.join()
                break
            except RuntimeError as err:
                print('wait a sec')
                time.sleep(3)
                pass
        while 1:
            try:
                thread = move_drone('Drone1', -45, 0, 0, 90, 5, is_async=True)
                time.sleep(15)
                thread.join()
                break
            except RuntimeError as err:
                print('wait a sec')
                time.sleep(3)
                pass

def control_drone1_pic(is_stop):
    while 1:
        time.sleep(1)  # fps = 1
        # prevent RPC error stop the process
        print('take_picture')
        responses = None
        try:
            responses = take_picture('Drone1', is_save=False)
        except RPCError as err:
            print('RPCError but it is okay: {}'.format(err))
        except Exception as exp:
            print('Exception but it is okay: {}'.format(exp))
        except RuntimeError as err:
            print('RuntimeError but it is okay: {}'.format(err))
        if responses:
            try:
                send_image_async(responses[0].image_data_uint8, '172.17.20.12', 18800)
            except RPCError as err:
                print('RPCError but it is okay: {}'.format(err))
            except Exception as exp:
                print('Exception but it is okay: {}'.format(exp))
            except RuntimeError as err:
                print('RuntimeError but it is okay: {}'.format(err))
            if is_stop.isSet():
                break

def control_drone2(is_follow):
    while 1:
        if is_follow.isSet():
            next_pos = get_cur_pos(vehicle_name='Drone1')
            rc = move_to_pos('Drone2', next_pos.x_val, next_pos.y_val, next_pos.z_val-1, 0, 10)
            responses = take_picture('Drone2')
            next_pos = get_cur_pos(vehicle_name='Drone1')
            rc = move_to_pos('Drone2', next_pos.x_val+10, next_pos.y_val, next_pos.z_val-1, 0, 1)
            responses = take_picture('Drone2')
            break
        else:
            time.sleep(1)


worker0 = threading.Thread(
    target=actuator, args=(server,), name='actuator')
worker1 = threading.Thread(
    target=control_drone1_move, args=(is_stop,), name='control_drone1_move')
worker2 = threading.Thread(
    target=control_drone1_pic, args=(is_stop,), name='control_drone1_pic')
worker3 = threading.Thread(
    target=control_drone2, args=(is_follow,), name='control_drone2')

print('Start worker threads')
worker0.start()
worker1.start()
worker2.start()
worker3.start()

print('Waiting for worker threads')
worker2.join()
worker3.join()
server.stop()
worker1.join()
worker0.join()




airsim.wait_key('Press any key to reset to original state')
client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")
