import kachaka_api
import os
import math

kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")
client.move_forward(-0.75)
client.rotate_in_place(-math.pi/2)
client.move_forward(0.80)
client.rotate_in_place(-math.pi/2)
client.move_forward(-0.75)
