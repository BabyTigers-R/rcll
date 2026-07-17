import kachaka_api
import os
import time

kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")
client.move_to_pose(0.344, 1.254, 1.713)
client.move_to_pose(0.542, 1.980, 1.604)
client.move_to_pose(0.484, 1.154, -0.682)
client.move_to_pose(1.430, 1.260, 0.941)
client.move_to_pose(2.088, 1.962, 3.084)
client.move_to_pose(1.419, 1.960, -1.632)
