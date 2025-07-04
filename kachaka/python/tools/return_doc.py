from kachaka_api import KachakaApiClient
import math
import os


kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")
client.return_home()
