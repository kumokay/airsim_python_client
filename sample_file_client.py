import msgpackrpc
import base64
import time

for i in range(3, 4):
    filepath = 'C:\\NESLProjects\\airsim_v1.2.0\\screenshot\\Drone1\\{}-0.png'.format(i)
    with open(filepath, 'rb') as binary_file:
        # Read the whole file at once
        data = binary_file.read()
    data = base64.b64encode(data)
    print(i)
    t1 = time.time()
    client = msgpackrpc.Client(msgpackrpc.Address("172.17.20.12", 18800))
    result = client.call('push', data, time.time())
    t2 = time.time()
    print(result)
    # print(t2-t1)
    # time.sleep(3)