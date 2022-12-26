# CickengateController

Contraption that opens or closes the gate of a chicken coop. It uses node red as brain and mqtt as comunication protocol. The gate will open and close depending on the position of the sun. You can define a time range for opening and closing. This range will be used if the sunrise or sunset is outside of this range. The gate will be controlled by these times when sunrise/sunset is not in this rage. [You can change the sunrise/sunset event to your needs](https://github.com/rdmtc/node-red-contrib-sun-position/wiki/Base-Functions#sun-times).

<img src="https://user-images.githubusercontent.com/87583841/177513696-c317f795-c482-4f3a-914d-01099f40bae9.png"  width="300" align="right">

Example settings:
- open from: 6:00 AM
- open until: 8:00 AM
- close from: 5:30 PM
- close until: 7:30 PM

Example cases:
- Sunrise @ 5:40 AM, gate opens @ 6:00 AM
- Sunrise @ 7:20 AM, gate opens @ 7:20 AM
- Sunrise @ 8:10 AM, gate opens @ 8:00 AM

## Main components
- Linear Actuator with integrated rotary encoder
- ESP32 Matrix Core + Wifi antenna
- VNH3ASP30-E a full-bridge motor driver
- DC-DC Stepdown
- Waterproof Connectors TE Connectivity AMP Superseal 1.5
- 3D Printed and sealed Housing

<img src="https://user-images.githubusercontent.com/87583841/177508631-c604d70c-6840-4101-81b9-23fd38725103.png" width="1000">
<img src="https://user-images.githubusercontent.com/87583841/177508322-37b72492-7daa-4d70-b307-69d66dd07091.jpg" width="1000">

## Hardware dependencies
<img src="https://user-images.githubusercontent.com/87583841/177510831-5efa2f2f-81bf-4123-974b-35eee85741f6.png" width="1000">
* Current sense output of the motor driver is also filtered by a lowpass.

## Webinterface
<img src="https://user-images.githubusercontent.com/87583841/177512667-4377206e-3c97-492d-b42d-e400036e0bc6.PNG" width="1000">

## NodeRed and MQTT
<img src="https://user-images.githubusercontent.com/87583841/177511456-48f2e264-8f94-4337-a0d1-d63007fbd9b6.png" width="200">

### Required NodeRed Nodes
- [node-red-contrib-sun-position](https://flows.nodered.org/node/node-red-contrib-sun-position)
- [node-red-dashboard](https://flows.nodered.org/node/node-red-dashboard)
- [node-red-contrib-ui-led](https://flows.nodered.org/node/node-red-contrib-ui-led)

Example docker-compose.yml file.
Example MQTT config
