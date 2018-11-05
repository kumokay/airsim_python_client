# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim
import numpy as np

import time
import logging

from helper.screenshot import screenshotHelper

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger()

def print_pos(drone_name):
	cur_state = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
	cur_pos = cur_state.position
	log.info('{} @ ({}, {}, {})'.format(drone_name, cur_pos.x_val, cur_pos.y_val, cur_pos.z_val))

	cur_pos = client.simGetObjectPose(drone_name).position
	log.debug('{} @ ({}, {}, {})'.format(drone_name, cur_pos.x_val, cur_pos.y_val, cur_pos.z_val))


def move_drone(drone_name, dx, dy, dz, yaw=0):
	print_pos(drone_name)
	# move the drong
	cur_state = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
	cur_pos = cur_state.position
	next_pos = airsim.Vector3r(cur_pos.x_val + dx, cur_pos.y_val + dy, cur_pos.z_val + dz)
	log.info("try to move: {} -> {}".format(cur_pos, next_pos))

	rc = client.moveToPositionAsync(
		next_pos.x_val, next_pos.y_val, next_pos.z_val, 1,
		yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw), 
		drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
		vehicle_name=drone_name).join()

	print_pos(drone_name)


client = airsim.MultirotorClient()
client.confirmConnection()

camera_dict = {}

for i in range(0, 4):
	drone_name = 'Drone{}'.format(i)
	camera_dict[drone_name] = screenshotHelper(drone_name, client, 'C:\\NESLProjects\\airsim_python_client\\airsim_exes\\CityEnviron\\screenshot\\')

	client.enableApiControl(True, vehicle_name=drone_name)
	client.armDisarm(True, vehicle_name=drone_name)

	cur_state = client.getMultirotorState(vehicle_name=drone_name)
	landed = cur_state.landed_state
	if landed == airsim.LandedState.Landed:
		client.takeoffAsync(vehicle_name=drone_name).join()
	else:
		client.hoverAsync(vehicle_name=drone_name).join()
	log.info('{} took off'.format(drone_name))

	move_drone(drone_name, 0, 0, -1, yaw=-60+30*i)
	print_pos(drone_name)


for r in range(0, 50):
	for i in range(0, 4):
		drone_name = 'Drone{}'.format(i)
		camera_dict[drone_name].do_screenshot(is_display=False)
	time.sleep(1)

