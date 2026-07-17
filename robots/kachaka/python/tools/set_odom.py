import kachaka_api
import os
 
kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")
 
client.set_robot_pose({ "x": 2.5, "y": 0.0, "theta": 0.0 })

