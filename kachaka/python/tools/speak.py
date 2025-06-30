import kachaka_api
import os
 
kachakaIP = os.getenv('kachaka_IP')
client = kachaka_api.KachakaApiClient(target=kachakaIP+":26400")
 
client.speak("こんにちは、ぼく，カチャカです")
