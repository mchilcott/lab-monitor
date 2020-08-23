
import Bristol621
import time
import numpy as np

import json
import paho.mqtt.client as mqtt


#########################
# Configure the wavemeter

device = Bristol621.Wavemeter('/dev/ttyUSB1')
lambda_units = 'GHz'
device.set_wavelength_units(lambda_units)
device.set_medium('vacuum') # Don't actually need this for GHz

samples = 20 

#########################
# Configure the MQTT client

mqtt_server = "hugin.px.otago.ac.nz"
mqtt_port = 1883

mqtt_user = None
mqtt_pass = None

mqtt_topic_base = "sensor/wavemeter"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    
client = mqtt.Client()
client.on_connect = on_connect

if mqtt_user is not None:
    client.username_pw_set(mqtt_user, mqtt_pass)

client.connect(mqtt_server, mqtt_port, 60)
client.loop_start()

#########################
# Collect data from the wavemeter, and subit it via MQTT

try:
    while True:
        l = []
        p = []
        for i in range(samples):
            w = device.get_wavelength()
            if w < 1:
                # Invalid value e.g. no laser detected
                continue
            l.append(w)

            t = device.get_power()
            p.append(t) 
            time.sleep(0.5)
        
        data_lambda = {'mean': np.mean(l), 'std': np.std(l), 'min': np.min(l), 'max': np.max(l), 'units':lambda_units}
        client.publish(mqtt_topic_base+'/frequency', json.dumps(data_lambda))
        
        
        data_power = {'mean': np.mean(p), 'std': np.std(p), 'min': np.min(p), 'max': np.max(p), 'units':'mW'}
        client.publish(mqtt_topic_base+'/power', json.dumps(data_power))
except:
    client.loop_stop()
    raise
