# Arduino-ESP8266-Pool-Manager
Pool manager with Phone App, Alexa, using Arduino and ESP8266

Project Summary:
This project is to allow remote control of my pool and associated lights with a dedicated display in the house, a phone application and Amazon Alexa. Any and/or all of these input devices may be used to control the pool. The pool temperature and the status of the control devices are monitored as additional information.

Folder Information:

123D Design: Contains Nextion 3.5" Display enclosure source file and .stl files of the bezel and base for 3D printing.
Note: the design uses 3M x 5mm brass inserts. 4 for the bezel, 4 for the base. Enclosure is sized to fit additional Modus Master PCB board which is the same size as the display using 3mm standoffs.

Alexa: Contains the AWS in teraction model and the lambda function (python).
Note: Need to set up an Amazon AWS account and a developer account. lambda function needs to be updated with your thingspeak channel and key details

Arduino: Contains the Modbus Slave and Modbus Master source file and the ESP8266Modbus library.
Note: All code has been compiled using Arduino IDE Ver 1.8.4. Use the arduino IDE boards manager to install the ESP8266 range of boards.
This project has specifically used an Aduino nano for the Modbus Slave and an ESP8266 NodeMCU for the Modbus Master.
All libraries are standard as referenced with the excpetion of the ESP8266Modbus library which is the referenced Arduino Modbus library modified (ie just commented out specific Arduino references) so that it compiles in the ESP8266 environment.

MIT_AI2_PhoneApp: Contains the MIT App Inventor 2 project source and .apk file for android phone.
Note: Project can be converted to Thinkable if apple app is required. Note Port 81 has been explicitly used for the phone app server.

Nextion Display: Contains the nextion editor project source and the loadable .tft file for nextion
Note: This project uses the Nextion 3.5" enhance display with RTC for time / timer functions.

PCB Designs: Contains the PCB gerber files and Altium Circuitmaker source files for the Modbus Slave, Modbus Master, Astralpools VX Chlorinator interface board and an optional Valve power supply. The project uses a 24VAC 20VA transformer which will support one valve. If two valves are required then you can use this PCB design with a 24VAC 50VA transformer from the same manufacturer.
Note: All PCB designs were made using Altium Circuitmaker. I used breadboardkiller to manufacture the boards for me. Wiring schematics are included in case anyone wishes to reproduce their own boards.

Thingspeak: Note this project uses two thingspeak channels as the IOT interface for Amazon Alex functionality, with the MQTT service for the publishing of status information and the subscibe service for the forwarding of Alexa command. The defualt in the code is for Alexa to be disabled. On configuration (ie. using Modbus Master board configuration pin), you enter your SSID, password details, along with the thinspeak channel details and tick the enable Alexa box.

Special thanks to those who provided the following libraries:

ModbusRTU: Samuel Marco i Armengol
https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino

WiFi Manager: Ken Taylor
https://github.com/kentaylor/WiFiManager

Alexa Belkin Switch WEMO Emulator: Brian Lough
https://github.com/witnessmenow/esp8266-alexa-wemo-emulator

MQTT Client: Nick O'Leary
https://github.com/knolleary/pubsubclient
