# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim
import numpy as np

client = airsim.MultirotorClient()
client.confirmConnection()

pose = client.simGetVehiclePose()
print("x={}, y={}, z={}".format(pose.position.x_val, pose.position.y_val, pose.position.z_val))

# for i in range(0, 77):
obj_name = 'Drone2'
pose = client.simGetObjectPose(obj_name);


pose.position = airsim.Vector3r(0, 4, 0)
success = client.simSetObjectPose(obj_name, pose, True)
print('success? {}'.format(success))

pose = client.simGetObjectPose(obj_name);
print("Position: {}, Orientation: {}".format((pose.position), (pose.orientation)))