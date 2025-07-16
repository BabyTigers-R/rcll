from kachaka_api import KachakaApiClient
import math
import os

kachakaIP = os.getenv('kachaka_IP')
client = KachakaApiClient(target=kachakaIP+":26400")

client.return_home()
