
import json
import paho.mqtt.client as mqtt
import time

# This script can be used to collect data from anything accessible to python.
#
# The purpose is to demonstrate how this can be done.


mqtt_server = ""
mqtt_port = 1883

mqtt_user = None
mqtt_pass = None

mqtt_topic = "sensor/defult/python_client"

delay_time = 1 # seconds

def get_value():
    """This is a dummy function, which should be used to collect data from a device.
    """
    
    return 0


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    
client = mqtt.Client()
client.on_connect = on_connect

if mqtt_user is not None:
    client.username_pw_set(mqtt_user, mqtt_pass)

client.connect(mqtt_server, mqtt_port, 60)


client.loop_start()
try:
    while True:
        # Get the value
        value = get_value();
        
        # Format the data message
        data = {"mean": value, "units": "V"}
        output_string = json.dumps(data)
        
        client.publish(mqtt_topic, output_string)
        
        # Wait, in the case that we 
        time.sleep(delay_time)
except:
    client.loop_stop()
    raise
