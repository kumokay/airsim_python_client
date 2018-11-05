# In settings.json first activate computer vision mode: 
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import setup_path 
import airsim

import time

client = airsim.MultirotorClient()
client.confirmConnection()

#Go to object in Unreal Editor, click on it and then look for Tags property. 
#Add a tag "MyObject" (without quotes), save and the query using following line
#see more about tags here: https://answers.unrealengine.com/questions/543807/whats-the-difference-between-tag-and-tag.html


pos_init = [
    (0, -40, -5),
    (0, -32, -5),
    (0, -24, -5),
    (0, -16, -5),
    (0, -8, -5),
    (0, -0, -5)]


for i in range(0, 6):
    vehicle_name = "Drone{}".format(i)
    client.enableApiControl(True, vehicle_name=vehicle_name)
    client.armDisarm(True, vehicle_name=vehicle_name)


for i in range(0, 6):

    vehicle_name = "Drone{}".format(i)
    pose_obj = client.simGetObjectPose(vehicle_name)
    # pose_obj = client.simGetVehiclePose(vehicle_name=vehicle_name)
    print('{} is at {}'.format(vehicle_name, pose_obj.position))

    pose_obj.position.x_val = pos_init[i][0]
    pose_obj.position.y_val = pos_init[i][1]
    pose_obj.position.z_val = pos_init[i][2]
    success = client.simSetObjectPose(vehicle_name, pose_obj, True)
    # time.sleep(1)
    pose_obj = client.simGetObjectPose(vehicle_name)
    print('{} is at {}'.format(vehicle_name, pose_obj.position))

airsim.wait_key("Success: {}")



airsim.wait_key("Success: {}")

for i in range(0, 6):
    vehicle_name = "Drone{}".format(i)
    multirotor_state = client.getMultirotorState(vehicle_name=vehicle_name)
    landed = multirotor_state.landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync(vehicle_name=vehicle_name).join()
    else:
        print("already flying...")
        client.hoverAsync(vehicle_name=vehicle_name).join()