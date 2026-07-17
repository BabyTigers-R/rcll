import os
import kachaka_api
import math
import matplotlib.pyplot as plt
import numpy as np
import cv2
from IPython.display import Image, display
from PIL import Image as PILImage
from io import BytesIO
import time

kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")

# show map id list
client.speak("ショウ，マップリスト")
map_list = client.get_map_list()
for map_list_entry in map_list:
    print("id:", map_list_entry.id)
    print("name:", map_list_entry.name)

print("previous map")
current_map_id = client.get_current_map_id()
print(current_map_id)


"""
# change the current map
client.speak("チェインジ，ザ・マップ")
# client.switch_map(map_list[-1].id, pose={"x": 0.0, "y": 0.0, "theta": 0.0})
client.switch_map(map_list[-2].id, pose={"x": 0.0, "y": 0.0, "theta": 0.0})

time.sleep(1)
print("new map")
"""
