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

# TARGET_MAP_PATH = "../../map/cyan-challenge.profile"
TARGET_MAP_PATH = "../../map/magenta-challenge.profile"

# show map id list
map_list = client.get_map_list()
for map_list_entry in map_list:
    print("id:", map_list_entry.id)
    print("name:", map_list_entry.name)

# import a map
# client.speak("インポート，ア・マップ")
# client.import_map(TARGET_MAP_PATH)
