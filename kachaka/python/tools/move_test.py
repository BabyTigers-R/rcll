import kachaka_api
import os

kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")
client.move_to_pose(0.13, 2.00, -0.07)
client.move_to_pose(0.43, 1.90, 0.01)
client.move_to_pose(0.60, 1.35, -0.92)
client.move_to_pose(1.41, 1.15, 0.68)
client.move_to_pose(1.90, 1.94, -3.0)
client.move_to_pose(1.59, 1.94, 3.13)



