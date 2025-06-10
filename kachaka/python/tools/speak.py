import kachaka_api
 
client = kachaka_api.KachakaApiClient(target="192.168.13.7:26400")
 
client.speak("こんにちは、ぼく，カチャカです")
