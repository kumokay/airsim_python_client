import setup_path 
import airsim
import numpy as np

from collections import defaultdict
import logging
import os
import threading
import time

from future.utils import iteritems


logging.basicConfig(level=logging.INFO)  # logging.DEBUG

log = logging.getLogger()


class ObjectHelper(object):
	# airsim city environment default parameters
	_MAX_CAR_NUM = 80  # should be 75; use 80 just in case
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

	def find_all_cars(self):
		for car_type in self._CAR_TYPE:
			for i in range(0, self._MAX_CAR_NUM):
				car_name = "{}_{}".format(car_type, i)
				pose = client.simGetObjectPose(car_name)
				if np.isnan(pose.position.x_val):
					if car_name in self.car_dict:
						del self.car_dict[car_name]
				else:
					self.car_dict[car_name] = pose.position

	def print_all_cars(self):
		log.debug('====== there are {} cars'.format(len(self.car_dict)))
		for car_name, car_pos in iteritems(self.car_dict):
			log.debug('{}: {}'.format(car_name, car_pos))

	def find_closeby_cars(self, drone_name, distance):
		result_list = []
		cur_pos = client.simGetObjectPose(drone_name).position
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


	def take_picture(self, drone_name, is_save=False):
		result_list = object_helper.find_closeby_cars(drone_name, 20)
		responses = client.simGetImages([
			airsim.ImageRequest("front_center", airsim.ImageType.Scene),
			# airsim.ImageRequest("bottom_center", airsim.ImageType.Scene),
		], vehicle_name=drone_name)
		if is_save:
			# save image
			drone_image_folder = '{}\\{}'.format(self._IMG_FOLDER, drone_name)
			if not os.path.isdir(drone_image_folder):
				os.makedirs(drone_image_folder)
			for idx, response in enumerate(responses):
				if response.compress: #png format
					log.debug('image type {}, size {}'.format(
						response.image_type, len(response.image_data_uint8)))
					if drone_name not in self._IMG_ID_DICT:
						self._IMG_ID_DICT[drone_name] = 0
					filename = '{}\\{}-{}.png'.format(
						drone_image_folder, self._IMG_ID_DICT[drone_name], idx)
					airsim.write_file(filename, response.image_data_uint8)
					log.info('save image: {}'.format(filename))
				else:
					log.debug('error: image format not support')
			# save result
			if result_list:
				filename = '{}\\{}.txt'.format(
					drone_image_folder, self._IMG_ID_DICT[drone_name])
				with open(filename, "w") as fd:
					for result_str in result_list:
						fd.write(result_str + '\n')
				log.info('found {} cars'.format(len(result_list)))
			self._IMG_ID_DICT[drone_name] += 1
		return responses


def track_all_cars(object_helper, is_stop):
	while not is_stop:
		object_helper.find_all_cars()
		time.sleep(0.1)


if __name__ == '__main__':

	client = airsim.MultirotorClient()
	client.confirmConnection()

	object_helper = ObjectHelper(client)
	client_helper = ClientHelper(client, object_helper)

	is_stop = threading.Event()

	worker_track_all_cars = threading.Thread(
		target=track_all_cars, 
		args=(object_helper, is_stop,), 
		name='track_all_cars')

	worker_track_all_cars.start()

	for i in range(0, 4):
		drone_name = 'Drone{}'.format(i)

		client.enableApiControl(True, vehicle_name=drone_name)
		client.armDisarm(True, vehicle_name=drone_name)

		cur_state = client.getMultirotorState(vehicle_name=drone_name)
		landed = cur_state.landed_state
		if landed == airsim.LandedState.Landed:
			client.takeoffAsync(vehicle_name=drone_name).join()
		else:
			client.hoverAsync(vehicle_name=drone_name).join()
		log.info('{} took off'.format(drone_name))

		client_helper.move_drone(drone_name, 0, 0, -5, yaw=0)
		client_helper.move_drone(drone_name, 0, -2, 0, yaw=0)
		# cam_orientation = airsim.to_quaternion(-0.4, 0, 0)  # https://quaternions.online/
		# client.simSetCameraOrientation(
		#	"front_center", cam_orientation, vehicle_name=drone_name)
		# client.moveByAngleZAsync(-0.524, 0, -1, 0, 1, vehicle_name=drone_name)
		client_helper.print_pos(drone_name)


	for r in range(0, 30):
		object_helper.find_all_cars()
		for i in range(0, 4):
			drone_name = 'Drone{}'.format(i)
			client_helper.take_picture(drone_name, is_save=True)
		time.sleep(0.5)

	is_stop.set()
	worker_track_all_cars.join()

