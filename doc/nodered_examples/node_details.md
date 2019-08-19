
# A useful NodeRED flow for getting details about all the nodes

The monitoring nodes output status information data regularly to `status/#`.

From the NodeRED menu, select Import -> Clipboard, and paste in the following code

```
[
    {
        "id": "4479dfb8.c361d8",
        "type": "tab",
        "label": "Node Details",
        "disabled": false,
        "info": ""
    },
    {
        "id": "ccc69fc1.e91278",
        "type": "mqtt in",
        "z": "4479dfb8.c361d8",
        "name": "",
        "topic": "status/#",
        "qos": "0",
        "broker": "46624d0f.ddd0a4",
        "x": 150,
        "y": 200,
        "wires": [
            [
                "57ab9195.41dec"
            ]
        ]
    },
    {
        "id": "1d2ecf79.1df7b9",
        "type": "debug",
        "z": "4479dfb8.c361d8",
        "name": "",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "true",
        "x": 810,
        "y": 200,
        "wires": []
    },
    {
        "id": "57ab9195.41dec",
        "type": "function",
        "z": "4479dfb8.c361d8",
        "name": "saver",
        "func": "var table = flow.get('table') || {};\n\nif (msg.topic){\n    var topic = msg.topic;\n    var payload = msg.payload;\n    table[topic] = payload;\n    flow.set('table', table);\n    return\n} else {\n    return {'payload': table};\n}",
        "outputs": 1,
        "noerr": 0,
        "x": 450,
        "y": 200,
        "wires": [
            [
                "1d2ecf79.1df7b9"
            ]
        ]
    },
    {
        "id": "b51daa64.d454c8",
        "type": "inject",
        "z": "4479dfb8.c361d8",
        "name": "",
        "topic": "",
        "payload": "DISPLAY",
        "payloadType": "str",
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "x": 220,
        "y": 100,
        "wires": [
            [
                "57ab9195.41dec"
            ]
        ]
    },
    {
        "id": "55452539.aea29c",
        "type": "comment",
        "z": "4479dfb8.c361d8",
        "name": "HOW TO USE",
        "info": "1) Click the button at the left of DISPLAY\n2) Open the debug tab to the right of the screen\n3) Inspect the last/new message on the debug screen\ni.e. look inside the payload\n4) This will show the most recent status messages for all the nodes",
        "x": 390,
        "y": 400,
        "wires": []
    },
    {
        "id": "46624d0f.ddd0a4",
        "type": "mqtt-broker",
        "z": "",
        "name": "",
        "broker": "monitoring_system_mosquitto_1",
        "port": "1883",
        "clientid": "",
        "usetls": false,
        "compatmode": true,
        "keepalive": "60",
        "cleansession": true,
        "birthTopic": "",
        "birthQos": "0",
        "birthPayload": "",
        "closeTopic": "",
        "closeQos": "0",
        "closePayload": "",
        "willTopic": "",
        "willQos": "0",
        "willPayload": ""
    }
]
```

You will then need to configure your MQTT connection with the correct broker.

Once this is imported, click on the HOW TO USE node for a quick guide.