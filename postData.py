import requests
import json

def post():
	with open ('sensorData.json') as sensorData:
		json_data = json.load(sensorData)
	try:
		r = requests.post('https://httpbin.org/post', data=json.dumps(json_data))
		print(r.text)
	except:
		print ("Something went wrong sending the request")
