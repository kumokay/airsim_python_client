import setup_path 
import airsim
import numpy as np

from collections import defaultdict
import logging
import os
import threading
import time

from future.utils import iteritems


logging.basicConfig(level=logging.DEBUG)  # logging.DEBUG

log = logging.getLogger()


class ObjectHelper(object):
	_FILE_FOLDER = (
		'C:\\NESLProjects\\airsim_python_client\\'
		'airsim_exes\\CityEnviron')

	# airsim city environment default parameters
	_MAX_CAR_NUM = 75  # should be 75; use 80 just in case
	_CAR_TYPE = {
		"Vehicle_Hatchback01_C",
		"Vehicle_Cupe01_C",
		"Vehicle_Saloon_C",
		"Vehicle_PoliceCruiser_C",
		"Vehicle_Sedan01_C",
		"Vehicle_SUV01_C",
	}

	def __init__(self, client):
		self.client = client
		self.car_dict = {}  # name: pos
		self.drone_dict = {}  # name: pos

	def find_all_drones(self):
		for i in range(0, 4):
			drone_name = 'Drone{}'.format(i)
			pose = client.simGetObjectPose(drone_name)
			self.drone_dict[drone_name] = pose.position

	def update_car_pos(self):
		for car_name in self.car_dict:
			pose = client.simGetObjectPose(car_name)
			self.car_dict[car_name] = pose.position

	def find_all_cars(self):
		if len(self._CAR_TYPE) == self._MAX_CAR_NUM:
			self.update_car_pos()
		else:
			for car_type in self._CAR_TYPE:
				for i in range(0, self._MAX_CAR_NUM):
					car_name = "{}_{}".format(car_type, i)
					pose = client.simGetObjectPose(car_name)
					if np.isnan(pose.position.x_val):
						if car_name in self.car_dict:
							del self.car_dict[car_name]
					else:
						self.car_dict[car_name] = pose.position

	def print_all_objects(self):
		log.debug('====== there are {} cars'.format(len(self.car_dict)))
		for drone_name, drone_pos in iteritems(self.drone_dict):
			log.debug('{}: {}'.format(drone_name, drone_pos))
		for car_name, car_pos in iteritems(self.car_dict):
			log.debug('{}: {}'.format(car_name, car_pos))

	def find_closeby_cars(self, drone_name, distance):
		result_list = []
		cur_pos = self.drone_dict[drone_name]
		log.info('=== cars close by {} ==='.format(drone_name))
		log.info('{} @ ({}, {}, {})'.format(
			drone_name, cur_pos.x_val, cur_pos.y_val, cur_pos.z_val))
		for car_name, car_pos in iteritems(self.car_dict):
			diff_x = abs(car_pos.x_val - cur_pos.x_val)
			diff_y = abs(car_pos.y_val - cur_pos.y_val)
			if diff_x < distance and diff_y < distance:
				result_str = '{}: {}, d={},{}'.format(
					car_name, car_pos, diff_x, diff_y)
				result_list.append(result_str)
		return result_list


class ClientHelper(object):
	_IMG_FOLDER = (
		'C:\\NESLProjects\\airsim_python_client\\'
		'airsim_exes\\CityEnviron\\screenshot')
	_IMG_ID_DICT = {}  # {drone_id: next_image_id}

	def __init__(self, client, object_helper):
		self.client = client
		self.object_helper = object_helper

	def print_pos(self, drone_name):
		cur_state = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
		cur_pos = cur_state.position
		log.info('{} @ ({}, {}, {})'.format(
			drone_name, cur_pos.x_val, cur_pos.y_val, cur_pos.z_val))
		cur_pos = client.simGetObjectPose(drone_name).position
		log.info('{} @ ({}, {}, {})'.format(
			drone_name, cur_pos.x_val, cur_pos.y_val, cur_pos.z_val))


	def move_drone(self, drone_name, dx, dy, dz, yaw=0):
		self.print_pos(drone_name)
		# move the drong
		cur_state = client.simGetGroundTruthKinematics(vehicle_name=drone_name)
		cur_pos = cur_state.position
		next_pos = airsim.Vector3r(
			cur_pos.x_val + dx, cur_pos.y_val + dy, cur_pos.z_val + dz)
		log.info("try to move: {} -> {}".format(cur_pos, next_pos))
		rc = client.moveToPositionAsync(
			next_pos.x_val, next_pos.y_val, next_pos.z_val, 1,
			yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw), 
			drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
			vehicle_name=drone_name).join()
		self.print_pos(drone_name)


	def take_picture(self, drone_list, is_save=False):
		result = {}
		for drone_name in drone_list:
			responses = client.simGetImages(
				[airsim.ImageRequest("front_center", airsim.ImageType.Scene),], 
				vehicle_name=drone_name)
			result[drone_name] = responses
		if is_save:
			for drone_name, responses in iteritems(result):
				drone_image_folder = '{}\\{}'.format(
					self._IMG_FOLDER, drone_name)
				if not os.path.isdir(drone_image_folder):
					os.makedirs(drone_image_folder)
				for idx, response in enumerate(responses):
					if not response.compress: #png format
						log.debug('error: image format not support')
						continue
					log.debug('image type {}, size {}'.format(
						response.image_type, len(response.image_data_uint8)))
					if drone_name not in self._IMG_ID_DICT:
						self._IMG_ID_DICT[drone_name] = 0
					filename = '{}\\{}-{}.png'.format(
						drone_image_folder, 
						self._IMG_ID_DICT[drone_name], idx)
					airsim.write_file(filename, response.image_data_uint8)
					log.info('save image: {}'.format(filename))
				self._IMG_ID_DICT[drone_name] += 1
		return result


if __name__ == '__main__':

	client = airsim.MultirotorClient()
	client.confirmConnection()

	object_helper = ObjectHelper(client)
	object_helper.find_all_drones()
	object_helper.find_all_cars()
	object_helper.print_all_objects()

	client_helper = ClientHelper(client, object_helper)

	drone_list = ['Drone{}'.format(i) for i in range(0, 4)]

	for drone_name in drone_list:
		client.enableApiControl(True, vehicle_name=drone_name)
		client.armDisarm(True, vehicle_name=drone_name)

		cur_state = client.getMultirotorState(vehicle_name=drone_name)
		landed = cur_state.landed_state
		if landed == airsim.LandedState.Landed:
			client.takeoffAsync(vehicle_name=drone_name).join()
		else:
			client.hoverAsync(vehicle_name=drone_name).join()
		log.info('{} took off'.format(drone_name))

		client_helper.move_drone(drone_name, 0, 0, -3, yaw=0)
		# client_helper.move_drone(drone_name, 0, 2, 0, yaw=0)
		# https://quaternions.online/
		cam_orientation = airsim.to_quaternion(-0.4, 0, 0)  
		client.simSetCameraOrientation(
			"front_center", cam_orientation, vehicle_name=drone_name)
		client_helper.print_pos(drone_name)

	for i in range(0, 30):
		log.debug('track_all_cars')
		object_helper.find_all_cars()
		client_helper.take_picture(drone_list, is_save=True)
		for drone_name in drone_list:
			result_list = object_helper.find_closeby_cars(drone_name, 20)
			if result_list:
				filename = '{}\\car_pos.txt'.format(object_helper._FILE_FOLDER)
				with open(filename, "a+") as fd:
					fd.write('[{}] {}\n'.format(i, drone_name))
					for result_str in result_list:
						fd.write('\t' + result_str + '\n')
				log.info('found {} cars near {}'.format(
					len(result_list), drone_name))
		time.sleep(1)

