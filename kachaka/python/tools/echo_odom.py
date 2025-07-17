import kachaka_api
import os
 
kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")

msg = client.get_robot_pose()
print(msg)
