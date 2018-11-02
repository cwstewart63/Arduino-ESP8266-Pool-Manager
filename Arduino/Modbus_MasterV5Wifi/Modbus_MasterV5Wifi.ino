/**
 *  Modbus master for NodeMCU with Nextion Display and LCD Display option.
 *  Alexa support, with Wemo emulator for control and interface to thingspeak for status updates
 * pin maping:
 * 0  - D3 - not used
 * 1  - TX - RS485 (MODBUS)
 * 3  - RX - RS485 (MODBUS)
 * 2  - D4 - BUILTIN_LED. Debug pin for Serial1 TX only.
 * 4  - D2 - I2C SDA (LCD)
 * 5  - D1 - I2C SCL (LCD)
 * 12 - D6 - TX - (SoftwareSerial) to Nextion RX (yellow wire)
 * 14 - D5 - RX - (SoftwareSerial) to Nextion TX (blue wire)
 * 13 - D7 - Trigger Input PIN for configuration of WiFi. 
 * 15 - D8 - Led for WiFi Connected
 * 16 - D0 - Led2 on NodeMCU. Used to indicate main program running. Flash every second. Solid on startup or config mode.
 * 
 */
 
// Modbus Libraries
#include <ESP8266ModbusRtu.h>       // https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino Library modified to run on ESP8266
#define MODBUSID   0                // Modbus Master Address
#define SERIALPORT   0        // serial port: 0-Serial, 1..3-Serial1..Serial3; 4: use software serial
#define TxEnable   0          // flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
#define BaudRate   9600       // set serial baudrate
#define Parity   SERIAL_8N1   // set serial parity
#define PollRate   350        // set polling rate msec
#define Version   10          // Version Number *10, 10 = Version 1.0
#define TRIGGER_PIN D7        // Trigger PIN to start WiFi Config Portal

// Nextion Libraries
#include <Nextion.h>        // https://github.com/itead/ITEADLIB_Arduino_Nextion
#include <SoftwareSerial.h>                   // Note also need to edit NexConfig.h to match
                                              //#include <SoftwareSerial.h>
                                              //extern SoftwareSerial HMISerial;
                                              //#define nexSerial HMISerial
SoftwareSerial HMISerial(D5, D6, false, 256); // Nextion TX(blue wire) to pin 14(D5) and RX (yellow wire) to pin 12(D6) of NodeMCU

// Libraries for WiFi / Web server
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <DNSServer.h>            // Comment out if using ESP8266mDNS.h
//#include <ESP8266mDNS.h>        // Future when mDNS supported on android / MIT AI2 or with Apple iOS (supported now)
#include <WiFiManager.h>          //https://github.com/kentaylor/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <FS.h>

// LCD Display Libraries
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4); // Check I2C address of LCD, normally 0x27 or 0x3F

                                  // Alexa Belkin Wemo Switch Emulator
#include "WemoSwitch.h"           // Alexa Libraries https://github.com/witnessmenow/esp8266-alexa-wemo-emulator
#include "WemoManager.h"
#include "CallbackFunction.h"

                                  // MQTT Library used to connect to Thingspeak for Alexa
#include <PubSubClient.h>         // https://github.com/knolleary/pubsubclient

// Alexa on/off callbacks
// Can optionally use Wemo switch to toggle the pool mode with either on/off call both active. If use this then MQTT subscribe is not necessary and
// then can remove the PoolModeIntent from the Alexa skill.
//void poolmodeOn();
//void poolmodeOff();
void spaOn();
void spaOff();
void gardenlightOn();
void gardenlightOff();
void poollightOn();
void poollightOff();
// Alexa Wemo Switch constructs
WemoManager wemoManager;
//WemoSwitch *poolmode = NULL;
WemoSwitch *spa = NULL;
WemoSwitch *gardenlight = NULL;
WemoSwitch *poollight = NULL;

// Wifi client for Thinspeak MQTT Connection
WiFiClient espclient;
PubSubClient mqttClient(espclient); // Initialize the PuBSubClient library.

//  
// Prototypes
//

// Handle messages from MQTT subscription.
int mqttSubscriptionCallback(char* topic, byte* payload, unsigned int meslength);  

// Genreate unique Client ID and connect to MQTT broker.
void mqConnect();  

// Subscribe to a field or feed from a ThingSpeak channel.
int mqttSubscribe( String subTopic );

// Publish messages to a channel feed.
void mqttPublish(String pubTopic, String dataString);

// Build a random client ID for MQTT connection
void getID(char clientID[], int idLength);

// Master registers: 
enum {
  SPA_VLV,          // Register 0 Pool/Spa Valve DO
  Spare_VLV,        // Register 1 Spare Valve DO
  Light_1,          // Register 2 Light 1 DO
  Light_2,          // Register 3 Light 2 DO
  PM_Button,        // Register 4 Pool Mode change button
  TIMESCH1,         // Register 5 Time Schedule 1 value
  TIMESCH2,         // Register 6 Time Schedule 2 value
  TIMESCH3,         // Register 7 Time Schedule 3 value
  TIMESCH4,         // Register 8 Time Schedule 4 value
  TIMESCH5,         // Register 9 Time Schedule 5 value  
  Temp_1,           // Register 10 Temperature 1 AI
  Temp_2,           // Register 11 Temperature 2 AI
  Pool_Mode,        // Register 12 Pool Mode Value 0=Auto, 1=ON, 2=OFF, 3=ERROR
  Pump,             // Register 13 Pump Running 0=OFF, 1=ON
  MSG_IN,           // Register 14 Slave Message IN Count
  MSG_OUT,          // Register 15 Slave Message OUT Count
  MSG_ERR,          // Register 16 Slave Message ERROR Count
  Display_Baud,     // Register 17 Slave Device Baud Rate
  Display_ID,       // Register 18 Slave Modubus Slave ID
  Display_Version,  // Register 19 Slave Software Version #
  Master_REGS        // Dummy register. using 0 offset to keep size of array
};

// data array for modbus network sharing
uint16_t au16data[Master_REGS];
uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query
unsigned long u32wait;
char buffer[10] = {0};  // char buffer for float to string conversion
float Temp1Val; //Temperature 1 value as float
int CurrentPage = 0;  // Create a variable to store which page is currently loaded
unsigned long previousMillis = 0;        // will store last time Nextion was updated
long delayTime = 1000;           // milliseconds of delay-time for Nextion refresh
uint32_t number1 = 0;     // variables to store current values. Only send if values change
uint32_t number2 = 0;
uint16_t old_Temp1 = 0;
uint16_t old_PM = 0;
uint16_t old_Pump = 0;


int ledState = LOW;     // use Builtin LED just to show loop code is running.
unsigned long previousMillis2 = 0;
const long interval = 1000;
String PM_TXT;            // variables for WiFi data
String Pool_TEMP;
String Pool_SPA;
String Pool_PUMP;
String wifitxt = "";
String configtxt = "";
String mactxt = "";
IPAddress myip; // declare the variable
char wifibuf[30] = {0};
char configbuf[30] = {0};
char macbuf[30] = {0};
uint8_t macAddr[6];

// ThingSpeak Settings. This data is set by the wifimanager configuration page
/*
 * Thingspeak Channel 1: Field1 = Pool_Temperature, Field2 = Pool_Mode, Field3 = Spa Status, Field4 = Pump Status
 * example                        22.5                       AUTO/ON/OFF         On/Off               On/Off
 * Thingspeak Channel 2: Field1 = Pool_Mode_Command
 *                                (0 = AUTO, 1=ON, 2=OFF)
 */
char writechannelID[7] = ""; //"123456" Thingspeak Channel 1 number
char readchannelID[7] = ""; //"654321" Thingspeak Channel 2 number
char writeAPIKey[17]  = ""; //"ABCDEF123456ABCD" write API key for your ThingSpeak Channel 1
char readAPIKey[17]  = ""; //"123456ABCDEF1234" read API key for your ThingSpeak Channel 2
char mqttUserName[] = "MQTTDemo-Pool";  // Can be any name.
char mqttPass[17] = ""; //"ABC123DEF456GH78" Change this your MQTT API Key from Account > MyProfile.
const char* mqttserver = "mqtt.thingspeak.com";
String topicString = "";
String data = "";
const int postingInterval = 60 * 1000; // post data every 60 seconds
unsigned long previousMillis3 = 0;
boolean disablecallback = false;
const int disableInterval = 10 * 1000; // disable data for 10 seconds on subscription
unsigned long previousMillis4 = 0;

// WiFi Manager Settings
const char* CONFIG_FILE = "/config.json";
bool initialConfig = false;
bool enableAlexa = false;
// Function Prototypes
bool readConfigFile();
bool writeConfigFile();

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(81);

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(MODBUSID,SERIALPORT,TxEnable); // this is master and RS-232 or USB-FTDI
/**
 * This is an structe which contains a query to an slave device
 */
modbus_t telegram[2];   // Number of MODBUS telegrams defined here

/*
 * Nextion Declarations
 * Declare a button object [page id:0,component id:1, component name: "b0"]. 
 */
NexPicture p4 = NexPicture(0, 41, "p4");
NexText t0 = NexText(0, 1, "t0");
NexText t1 = NexText(0, 2, "t1");
NexPicture p5 = NexPicture(0, 42, "p5");
NexText Ctxt = NexText(1, 16, "Ctxt");
NexText Wtxt = NexText(1, 17, "Wtxt");
NexText stxt = NexText(1, 18, "stxt");
NexText atxt = NexText(1, 19, "atxt");
NexPicture p0 = NexPicture(0, 3, "p0");
NexPicture p1 = NexPicture(0, 4, "p1");
NexPicture p2 = NexPicture(0, 5, "p2");
NexNumber n0 = NexNumber(1, 2, "n0");
NexNumber n1 = NexNumber(1, 3, "n1");
NexNumber n2 = NexNumber(1, 4, "n2");
NexNumber n3 = NexNumber(1, 5, "n3");
NexNumber n4 = NexNumber(1, 6, "n4");
NexNumber n5 = NexNumber(1, 7, "n5");
NexVariable gsched1 = NexVariable(0, 30, "gsched1");
NexVariable gsched2 = NexVariable(0, 49, "gsched2");
NexPage page0 = NexPage(0, 0, "page0");  // Page added as a touch event
NexPage page1 = NexPage(1, 0, "page1");  // Page added as a touch event

/*
 * Nextion touch events
 * Register object to the touch event list.  
 */
NexTouch *nex_listen_list[] = 
{
    &p4,
    &p0,
    &p1,
    &p2,
    &page0,  // Page added as a touch event
    &page1,  // Page added as a touch event
    &gsched1,  // Timer1 on/off change added as event
    &gsched2,  // Timer2 on/off change added as event
    NULL
};

/*
 * Nextion Callbacks
 * Button0 component pop callback function.
 * In this example,the value of the number component will plus one every time when button0 is released.
 */
// Callback event when timer1 changes.
void gsched1PopCallback(void *ptr)
{
    //Serial.println("gsched1PopCallback");
    gsched1.getValue(&number1);
    au16data[TIMESCH1] = number1;
}
// Callback event when timer2 changes.
void gsched2PopCallback(void *ptr)
{
    //Serial.println("gsched2PopCallback");
    gsched2.getValue(&number2);
    au16data[TIMESCH2] = number2;
}
// Callback event for Pool Mode Pushbutton
void p4PopCallback(void *ptr)
{
    //Serial.println("p4PopCallback");
    au16data[PM_Button] = 1;    
}
// Callback event for Pool/SPA valve Pushbutton
void p0PopCallback(void *ptr)
{
    uint32_t number = 0;
    //Serial.println("p0PopCallback");
    p0.getPic(&number);
    if (number == 1)
    {
        number = 2;
        au16data[SPA_VLV] = 1;    
    }
    else
    {
        number = 1;
        au16data[SPA_VLV] = 0;
    }
    p0.setPic(number);
}
// Callback event for Light 1 Pushbutton
void p1PopCallback(void *ptr)
{
    uint32_t number = 0;
    //Serial.println("p1PopCallback");
    p1.getPic(&number);
    if (number == 3)
    {
        number = 4;
        au16data[Light_1] = 1;    
    }
    else
    {
        number = 3;
        au16data[Light_1] = 0;
    }
    p1.setPic(number);
}
// Callback event for Light 2 Pushbutton
void p2PopCallback(void *ptr)
{
    uint32_t number = 0;
    //Serial.println("p2PopCallback");
    p2.getPic(&number);
    if (number == 3)
    {
        number = 4;
        au16data[Light_2] = 1;    
    }
    else
    {
        number = 3;
        au16data[Light_2] = 0;
    }
    p2.setPic(number);
}

// Page change event:
// As on each page change the data reverts to the default information then on each page change we must set the data again.
// To minimize the amount of data sent only changed data is sent, hence we must also resend on page change.
void page0PushCallback(void *ptr)  // If page 0 is loaded on the display, the following is going to execute:
{
  CurrentPage = 0;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display

  memset (buffer, 0, sizeof(buffer));
  Temp1Val = (au16data[Temp_1]) / 10.0;           // convert MODBUS integer to float
  dtostrf(Temp1Val, 4, 1, buffer);                // convert float to string with, min 4 char, 1 decimal point
  t1.setText(buffer);                             // send temperature value string to t1
  old_Temp1 = au16data[Temp_1];
  Pool_TEMP = buffer;
  if (au16data[Pool_Mode] == 0) {
    t0.setText("AUTO");
    PM_TXT = "AUTO";
  }
  if (au16data[Pool_Mode] == 1) {
    t0.setText("ON");
    PM_TXT = "ON";
  }
  if (au16data[Pool_Mode] == 2) { 
    t0.setText("OFF");
    PM_TXT = "OFF";
  }
  if (au16data[Pool_Mode] == 3) { 
    t0.setText("ERROR");
    PM_TXT = "ERROR";
  }
  old_PM = au16data[Pool_Mode];
  if (au16data[Pump] == 1) sendCommand("p5.pic=10");
  if (au16data[SPA_VLV] == 1) sendCommand("p0.pic=2");
  if (au16data[Light_1] == 1) sendCommand("p1.pic=4");
  if (au16data[Light_2] == 1) sendCommand("p2.pic=4");
    
  //Serial.println("page0PushCallback");
}  // End of press event


// Page change event:
// No need to resend data for Page 1 as it is sent every cycle anyway.
void page1PushCallback(void *ptr)  // If page 1 is loaded on the display, the following is going to execute:
{
  CurrentPage = 1;  // Set variable as 1 so from now on arduino knows page 1 is loaded on the display
  //Serial.println("page1PushCallback");
  //n3.setValue(BaudRate);
  //n4.setValue(MODBUSID);
  //n5.setValue(Version);
  Ctxt.setText(configbuf);
  Wtxt.setText(wifibuf);
  if (WiFi.status() == WL_CONNECTED) sendCommand("stxt.pco=2016");  // set WiFi text to green font if wifi is connected
  if (enableAlexa == true) sendCommand("atxt.pco=31");              // set Alexa text to blue font if Alexa functions enabled
}  // End of press event

// Alexa Wemo Function calls
// Poolmode calls only needed if using wemo switch instead of Alex skill
//void poolmodeOn() {
    //Serial.println("Pool Mode on ...");
    //au16data[PM_Button] = 1;
//}
//void poolmodeOff() {
    //Serial.println("Pool Mode off ...");
    //au16data[PM_Button] = 1;
//}
// For on calls set the modbus value to 1 and the nextion picture to the on state.
// For off calls set the modbus value to 0 and the nextion picture to the off state.
void spaOn() {
    //Serial.println("Spa on ...");
    au16data[SPA_VLV] = 1;
    p0.setPic(2);
}
void spaOff() {
    //Serial.println("Spa off ...");
    au16data[SPA_VLV] = 0;
    p0.setPic(1);
}
void gardenlightOn() {
    //Serial.println("Garden Lights on ...");
    au16data[Light_1] = 1;
    p1.setPic(4);
}
void gardenlightOff() {
    //Serial.println("Garden Lights off ...");
    au16data[Light_1] = 0;
    p1.setPic(3);
}
void poollightOn() {
    //Serial.println("Pool Lights on ...");
    au16data[Light_2] = 1;
    p2.setPic(4);
}
void poollightOff() {
    //Serial.println("Pool Lights off ...");
    au16data[Light_2] = 0;
    p2.setPic(3);
}
// Read the configuration file with the stored parameters from the wifi manager
// This contains the Thinspeak configuration parameters.
bool readConfigFile() {
  // this opens the config file in read-mode
  File f = SPIFFS.open(CONFIG_FILE, "r");
  
  if (!f) {
    Serial1.println("Configuration file not found");
    return false;
  } else {
    // we could open the file
    size_t size = f.size();
    // Allocate a buffer to store contents of the file.
    std::unique_ptr<char[]> buf(new char[size]);

    // Read and store file contents in buf
    f.readBytes(buf.get(), size);
    // Closing file
    f.close();
    // Using dynamic JSON buffer which is not the recommended memory model, but anyway
    // See https://github.com/bblanchon/ArduinoJson/wiki/Memory%20model
    DynamicJsonBuffer jsonBuffer;
    // Parse JSON string
    JsonObject& json = jsonBuffer.parseObject(buf.get());
    // Test if parsing succeeds.
    if (!json.success()) {
      Serial1.println("JSON parseObject() failed");
      return false;
    }
    //json.printTo(Serial);

    // Parse all config file parameters, override 
    // local config variables with parsed values
    if (json.containsKey("thingspeakApiKey")) {
      strcpy(writeAPIKey, json["thingspeakApiKey"]);      
    }
    if (json.containsKey("thingspeakchannel1")) {
      strcpy(writechannelID, json["thingspeakchannel1"]);      
    }
    if (json.containsKey("thingspeakchannel2")) {
      strcpy(readchannelID, json["thingspeakchannel2"]);      
    }
    if (json.containsKey("thingspeakmqttkey")) {
      strcpy(mqttPass, json["thingspeakmqttkey"]);      
    }
    if (json.containsKey("enableAlexa")) {
      enableAlexa = json["enableAlexa"];
    }

  }
  Serial1.println("\nConfig file was successfully parsed");
  return true;
}
// Write the configuration file with the stored parameters for thingspeak and alexa
// This writes the Thinspeak configuration parameters.
bool writeConfigFile() {
  Serial1.println("Saving config file");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();

  // JSONify local configuration parameters
  json["thingspeakApiKey"] = writeAPIKey;
  json["thingspeakchannel1"] = writechannelID;
  json["thingspeakchannel2"] = readchannelID;
  json["thingspeakmqttkey"] = mqttPass;
  json["enableAlexa"] = enableAlexa;

  // Open file for writing
  File f = SPIFFS.open(CONFIG_FILE, "w");
  if (!f) {
    Serial1.println("Failed to open config file for writing");
    return false;
  }

  //json.prettyPrintTo(Serial);
  // Write data to file and close it
  json.printTo(f);
  f.close();

  Serial1.println("\nConfig file was successfully saved");
  return true;
}
// This performs the MQTT connection request. If fails then just wait until next posting interval to retry.
void mqConnect()
{
    char clientID[ 9 ];
    
        getID(clientID,8);
       
        // Connect to the MQTT broker.
        Serial1.print( "Attempting MQTT connection..." );
        if ( mqttClient.connect( clientID, mqttUserName, mqttPass ) )
        {
            Serial1.println( "Connected with Client ID:  " + String( clientID ) + " User "+ String( mqttUserName ) + " Pwd "+String( mqttPass ) );
           
        } else
        {
            Serial1.print( "failed, rc = " );
            // See http://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
            Serial1.print( mqttClient.state() );
            //Serial.println( " Will try again in 5 seconds" );
            //delay( 5000 );
        }
}

/**
 * Build a random client ID
 *   clientID - char array for output
 *   idLength - length of clientID (actual length will be one longer for NULL)
 */

void getID(char clientID[], int idLength){
static const char alphanum[] ="0123456789"
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz";                        // For random generation of client ID.

    // Generate ClientID
    for (int i = 0; i < idLength ; i++) {
        clientID[ i ] = alphanum[ random( 51 ) ];
    }
    clientID[ idLength ] = '\0';
    
}
// MQTT Publish Request
void mqttPublish(String pubTopic, String dataString) {
        
    // publish data to ThingSpeak channel feed.

    mqttClient.publish( pubTopic.c_str(), dataString.c_str() );
    Serial1.println( "Pubished to channel " + dataString  );
}

/**
 * MQTT Subscribe to fields of a channel
 */
 
int mqttSubscribe( String subTopic ){
        
    Serial1.println( "Subscribing to " +subTopic );
    //Serial.println( "State= " + String( mqttClient.state() ) );
    char charBuf[ subTopic.length()+1 ];
    subTopic.toCharArray( charBuf,subTopic.length()+1 );
    
    return mqttClient.subscribe( charBuf ,0 );
    
}
// MQTT Callback Function which executes on each receipt of subscibed data
int mqttSubscriptionCallback( char* topic, byte* payload, unsigned int mesLength ) {

  char p[mesLength + 1];
  memcpy( p, payload, mesLength );
  p[mesLength] = NULL;
  //Serial.print("Message arrived [");
  //Serial.print(topic);
  //Serial.print("] ");
  //Serial.print( String(p) );
  //Serial.println();
  // As on each initial subcription the current data is sent. Ignore this first message, as we want to process only actual new requests
  // Disable this for 10sec on each subscription.
  if (disablecallback == true) Serial1.println("Callback Disabled on Startup");
  // Pool Mode toggles from Auto to On to Off so depending on the current state we need to pulse one or two times.
  if (disablecallback == false) {
    if ((PM_TXT == "AUTO") && (p[0] == '1')) {    // Example: Current Pool Mode is Auto, Command is On.
      //Serial.println("AUTO Pulse one to ON");
      au16data[PM_Button] = 1;
    }
    if ((PM_TXT == "AUTO") && (p[0] == '2')) {
      //Serial.println("AUTO Pulse two to OFF");
      au16data[PM_Button] = 2;
    }
    if ((PM_TXT == "ON") && (p[0] == '2')) {
      //Serial.println("ON Pulse one to OFF");
      au16data[PM_Button] = 1;
    }
    if ((PM_TXT == "ON") && (p[0] == '0')) {
      //Serial.println("ON Pulse two to AUTO");
      au16data[PM_Button] = 2;
    }
    if ((PM_TXT == "OFF") && (p[0] == '0')) {
      //Serial.println("OFF Pulse one to AUTO");
      au16data[PM_Button] = 1;
    }
    if ((PM_TXT == "OFF") && (p[0] == '1')) {
      //Serial.println("OFF Pulse two to ON");
      au16data[PM_Button] = 2;
    }  
  }
}
  
void setup(void)
{     // MODBUS Master setup
      // telegram 0: read input registers
    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = 4; // function code (this one is input registers read)
    telegram[0].u16RegAdd = 13; // start address in slave
    telegram[0].u16CoilsNo = 10; // number of elements (coils or registers) to read
    telegram[0].au16reg = au16data+10; // pointer to a memory array in the Arduino

      // telegram 1: write a single register
    telegram[1].u8id = 1; // slave address
    telegram[1].u8fct = 16; // function code (this one is write a multiple register)
    telegram[1].u16RegAdd = 3; // start address in slave
    telegram[1].u16CoilsNo = 10; // number of elements (coils or registers) to write
    telegram[1].au16reg = au16data; // pointer to a memory array in the Arduino

    master.begin( BaudRate, Parity ); // baud-rate at 9600
    master.setTimeOut( 2000 ); // if there is no answer in 2000 ms, roll over
    u32wait = millis() + PollRate; // MODBUS message sent every 350ms
    u8state = u8query = 0; 
    
    // Nextion Display Setup
    /* Set the baudrate which is for debug and communicate with Nextion screen. */
    nexInit();
    //Serial.begin(9600);
    //HMISerial.begin(9600);
    delay(1000);  // This delay is just in case the nextion display didn't start yet, to be sure it will receive the following command.
    //sendCommand("baud=57600");  // Set new baud rate of nextion to 57600
    //HMISerial.end();  // End the serial comunication of baud=9600

    HMISerial.begin(57600);   // Start serial comunication at baud=57600. Need to match in Nextion hardware.cpp and set baud=57600 in nextion main pre-initialize
                              // Alternatively use the sendcommand above to nextion to change the default baud rate. Not preferred option.
    
    /* Register the pop event callback function of the current button0 component. */
    p4.attachPop(p4PopCallback);
    p0.attachPop(p0PopCallback);
    p1.attachPop(p1PopCallback);
    p2.attachPop(p2PopCallback);
    page0.attachPush(page0PushCallback);  // Page press event
    page1.attachPush(page1PushCallback);  // Page press event
    gsched1.attachPop(gsched1PopCallback);
    gsched2.attachPop(gsched2PopCallback);

    // put your setup code here, to run once:
    Serial1.begin(115200); //Using D4 Serial1 as a debug pin
    //Serial.println("\n Starting");

    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    pinMode(D0, OUTPUT);  // Program Running Indicator
    pinMode(D8, OUTPUT);  // WiFi Connected Indicator

    // Mount the filesystem
    bool result = SPIFFS.begin();
    //Serial.println("SPIFFS opened: " + result);

    if (!readConfigFile()) {
      Serial1.println("Failed to read configuration file, using default values");
    }

    if (WiFi.SSID()=="") configtxt = "Configure WiFi";
    else configtxt = "WiFi Configured";
    
    
    WiFi.mode(WIFI_STA); // Force to station mode because if device was switched off while in access point mode it will start up next time in access point mode.
      
      // Start the mDNS URL is esp8266.local
      //if (!MDNS.begin("esp8266"))   {  Serial.println("Error setting up MDNS responder!");  }
      //else                          {  Serial.println("mDNS responder started");  }
      
      // Start the server
   server.begin();
   WiFi.setAutoReconnect(true);
   Serial1.println("Server started");
   Serial1.println("Connecting");
   WiFi.waitForConnectResult(); 
   delay(5000);
   if (WiFi.status() == WL_CONNECTED) {
     Serial1.print("Connected IP: ");
     Serial1.println(WiFi.localIP());
     Serial1.print("MAC Address: ");
     Serial1.println(WiFi.macAddress());
     myip = WiFi.localIP();
     wifitxt = "IP: " + String(myip[0]) + "." + String(myip[1]) + "." + String(myip[2]) + "." + String(myip[3]);
     WiFi.macAddress(macAddr);
     mactxt = String(macAddr[0]) + ":" + String(macAddr[1]) + ":" + String(macAddr[2]) + ":" + String(macAddr[3]) + ":" + String(macAddr[4]) + ":" + String(macAddr[5]);
   }
   else { 
     Serial1.println("Connection Failed");
     wifitxt = "Connection Failed";
   }

    // WiFi connection result is saved for display on Nextion and/or LCD display
    wifitxt.toCharArray(wifibuf,30);
    configtxt.toCharArray(configbuf,30);
    mactxt.toCharArray(macbuf,30);

    lcd.begin(D2,D1);      // In ESP8266-01, SDA=D2, SCL=D1 
    lcd.backlight();
    lcd.setCursor(4, 0);
    lcd.print("LeisurePools");
    lcd.setCursor(0, 1);
    lcd.print(configtxt);
    lcd.setCursor(0, 2);
    lcd.print(wifitxt);
    lcd.setCursor(0, 3);
    lcd.print(WiFi.macAddress());

    // MQTT Setup
    mqttClient.setServer(mqttserver, 1883);   // Set the MQTT broker details.
    mqttClient.setCallback( mqttSubscriptionCallback );   // Set the MQTT message handler function.
    // Wemo setup
    wemoManager.begin();
    Serial1.println("Starting WemoManager");
    // Format: Alexa invocation name, local port no, on callback, off callback
    //poolmode = new WemoSwitch("pool mode", 82, poolmodeOn, poolmodeOff);
    spa = new WemoSwitch("spa", 83, spaOn, spaOff);
    gardenlight = new WemoSwitch("garden lights", 84, gardenlightOn, gardenlightOff);
    poollight = new WemoSwitch("pool lights", 85, poollightOn, poollightOff);
    //wemoManager.addDevice(*poolmode);
    wemoManager.addDevice(*spa);
    wemoManager.addDevice(*gardenlight);
    wemoManager.addDevice(*poollight);

    Serial1.println("setup done");
}

void loop(void)
{   
    nexLoop(nex_listen_list);

    // is configuration portal requested?
    // use configuration portal if requested by trigger pin
    if ( digitalRead(TRIGGER_PIN) == LOW ) {
      Serial1.println("Configure WiFi connection now");
      digitalWrite(D0, LOW);  // set the configuration LED to on.

      // Extra parameters to be configured
      // After connecting, parameter.getValue() will get you the configured value
      // Format: <ID> <Placeholder text> <default value> <length> <custom HTML> <label placement>

      // Thingspeak API Key - this is a straight forward string parameter
      WiFiManagerParameter p_thingspeakApiKey("thingspeakapikey", "Thingspeak API Key", writeAPIKey, 17);
      // Thingspeak Write Channel ID - this is a straight forward string parameter
      WiFiManagerParameter p_thingspeakchannel1("thingspeakchannel1", "Thingspeak Write Channel", writechannelID, 7);
      // Thingspeak Read Channel ID - this is a straight forward string parameter
      WiFiManagerParameter p_thingspeakchannel2("thingspeakchannel2", "Thingspeak Read Channel", readchannelID, 7);
      // Thingspeak MQTT Key - this is a straight forward string parameter
      WiFiManagerParameter p_thingspeakMQTTkey("thingspeakmqttkey", "Thingspeak MQTT Key", mqttPass, 17);

      // enableAlexa or not - bool parameter visualized using checkbox, so couple of things to note
      // - value is always 'T' for true. When the HTML form is submitted this is the value that will be 
      //   sent as a parameter. When unchecked, nothing will be sent by the HTML standard.
      // - customhtml must be 'type="checkbox"' for obvious reasons. When the default is checked
      //   append 'checked' too
      // - labelplacement parameter is WFM_LABEL_AFTER for checkboxes as label has to be placed after the input field

      char customhtml[24] = "type=\"checkbox\"";
      if (enableAlexa) {
        strcat(customhtml, " checked");
      }
      WiFiManagerParameter p_enableAlexa("enableAlexa", "Alexa Enable", "T", 2, customhtml, WFM_LABEL_AFTER);

      // Just a quick hint
      WiFiManagerParameter p_hint("<small>*Hint: if you want to reuse the currently active WiFi credentials, leave SSID and Password fields empty</small>");
            
      //WiFiManager
      //Local intialization. Once its business is done, there is no need to keep it around
      WiFiManager wifiManager;
      wifiManager.setDebugOutput(false);  // disable WM: debug messages
      //reset settings - for testing
      //wifiManager.resetSettings();

      //sets timeout until configuration portal gets turned off
      //useful to make it all retry or go to sleep
      //in seconds
      wifiManager.setConfigPortalTimeout(180);

      //add all parameters here
    
      wifiManager.addParameter(&p_hint);
      wifiManager.addParameter(&p_thingspeakApiKey);
      wifiManager.addParameter(&p_thingspeakchannel1);
      wifiManager.addParameter(&p_thingspeakchannel2);
      wifiManager.addParameter(&p_thingspeakMQTTkey);
      wifiManager.addParameter(&p_enableAlexa);

      //it starts an access point with the specified name
      //here  "AutoConnectAP"
      //and goes into a blocking loop awaiting configuration

      //WITHOUT THIS THE AP DOES NOT SEEM TO WORK PROPERLY WITH SDK 1.5 , update to at least 1.5.1
      //WiFi.mode(WIFI_STA);
      if (!wifiManager.startConfigPortal("LeisureAP")) {
        //Serial.println("Configuration Failed / Timeout");
        configtxt = "Configure WiFi Failed"; 
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        //ESP.reset();
        //delay(5000);
      }

      //if you get here you have connected to the WiFi
      else {
        // Getting posted form values and overriding local variables parameters
        // Config file is written regardless the connection state
        strcpy(writeAPIKey, p_thingspeakApiKey.getValue());
        strcpy(writechannelID, p_thingspeakchannel1.getValue());
        strcpy(readchannelID, p_thingspeakchannel2.getValue());
        strcpy(mqttPass, p_thingspeakMQTTkey.getValue());
        enableAlexa = (strncmp(p_enableAlexa.getValue(), "T", 1) == 0);
        writeConfigFile();
        //Serial.print("API Key is : ");
        //Serial.println(writeAPIKey);
        //Serial.print("Channel1 is : ");
        //Serial.println(writechannelID);
        //Serial.print("Channel2 is : ");
        //Serial.println(readchannelID);
        //Serial.print("Enable Alexa is : ");
        //Serial.println(enableAlexa);
        Serial1.println("Configuration Complete");
      }
      ESP.reset(); // This is a bit crude. For some unknown reason webserver can only be started once per boot up 
      // so resetting the device allows to go back into config mode again when it reboots.
      delay(5000);
    
    } 

    // Alexa Wemo Manager. Only run wemo serverloop if Alexa option is enabled.
    if (enableAlexa == true) wemoManager.serverLoop();

    // Cycle Builtin LED to show main program running
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis2 >= interval) {
      previousMillis2 = currentMillis;   
      if (ledState == LOW)
        ledState = HIGH;  // Note that this switches the LED *off*
      else
        ledState = LOW;   // Note that this switches the LED *on*
      digitalWrite(D0, ledState);
    }
    
    // D8 used to show WiFi connection is good.
    if (WiFi.status() == WL_CONNECTED) digitalWrite(D8, HIGH);
    else digitalWrite(D8, LOW);  
    
    // MODBUS Master. This cycles the modbus messages and polls for the responses.
    switch( u8state ) {
    case 0: 
      if (millis() > u32wait) u8state++; // wait state
      break;
    case 1:   
      master.query( telegram[u8query] ); // send query (only once)
      u8state++;
      // The Pool Mode button value is reset to 0 when the message is sent. This simulates a momentary button but also allows multiple pulses when the value is > 1.
      // For the Alexa skill a value of 2 is needed.
      if ((u8query == 1) && (au16data[PM_Button] >= 1)) au16data[PM_Button] =0; // reset PM_Button to 0 only once telegram is sent
    u8query++;
    if (u8query == 2) u8query = 0;
      break;
    case 2:
      master.poll(); // check incoming messages
      if (master.getState() == COM_IDLE) {
        u8state = 0;
        u32wait = millis() + PollRate; // After message response reset the delay before new message is sent (ie modbus nessage cycle time)
      }
      break;
    }
    
     /* NEXTION Display
     * When a pop or push event occured every time, 
     * the corresponding component[right page id and component id] in touch event list will be asked.
     * The delay time of 1000ms is used to limit the data being sent to nextion and avoid conflict with touch events
     */
    
    if((millis() - previousMillis >= delayTime)) {
  
      previousMillis = millis();  // Remember the time
    
      if(CurrentPage == 0){  // If the display is on page 0, do the following:
        memset (buffer, 0, sizeof(buffer));
        //au16data[Temp_1] = random(100,300);           // test value
        if (au16data[Temp_1] != old_Temp1) {              // only send data if it has changed value
          Temp1Val = (au16data[Temp_1]) / 10.0;           // convert MODBUS integer to float
          dtostrf(Temp1Val, 4, 1, buffer);                // convert float to string with, min 4 char, 1 decimal point
          t1.setText(buffer);                             // send temperature value string to t1
          old_Temp1 = au16data[Temp_1];
          Pool_TEMP = buffer;
        }        
        //au16data[Pool_Mode] = random(0,3);          // test value
        if (au16data[Pool_Mode] != old_PM) {              // only send data if it has changed value
          if (au16data[Pool_Mode] == 0) {
            t0.setText("AUTO");
            PM_TXT = "AUTO";
          }
          if (au16data[Pool_Mode] == 1) {
            t0.setText("ON");
            PM_TXT = "ON";
          }
          if (au16data[Pool_Mode] == 2) { 
            t0.setText("OFF");
            PM_TXT = "OFF";
          }
          if (au16data[Pool_Mode] == 3) { 
            t0.setText("ERROR");
            PM_TXT = "ERROR";
          }
          old_PM = au16data[Pool_Mode];
        }
        if (au16data[Pump] != old_Pump) {              // only send data if it has changed value
          if (au16data[Pump] == 0) sendCommand("p5.pic=9");
          if (au16data[Pump] == 1) sendCommand("p5.pic=10");
          old_Pump = au16data[Pump];
        }
        if (au16data[SPA_VLV] == 1) Pool_SPA = "on";
        else Pool_SPA = "off";
        if (au16data[Pump] == 1) Pool_PUMP = "on";
        else Pool_PUMP = "off";
      }

      if(CurrentPage == 1){  // If the display is on page 1, do the following:
        n0.setValue(master.getOutCnt());
        n1.setValue(master.getInCnt());
        n2.setValue(master.getErrCnt());
        
      }
   
    }
    //delay(50);  // small delay to allow nextion data to be sent prior to next nextion event listen
        
    // Write data to thingspeak if alexa is enabled. Note the thingspeak channel ID's and Keys must also be set and valid.
    // Posting interval is set to every 60 seconds
    if((millis() - previousMillis3 >= postingInterval) && (enableAlexa == true)) {
      previousMillis3 = millis();  // Remember the time
   
      // If not connected to MQTT then try to connect and subcribe to the command channel
      if (!mqttClient.connected())
      {
       
       mqConnect(); // Connect if MQTT client if not connected.
        
         topicString = "channels/" + String( readchannelID ) + "/subscribe/fields/field1";
         //topicString = "channels/" + String( readchannelID ) + "/subscribe/fields/field1/" + String(readAPIKey);
         //topicString = "channels/" + String( readchannelID ) + "/subscribe/json";
         //topicString = "channels/" + String( readchannelID ) + "/subscribe/json/" + String(readAPIKey);
         if(mqttSubscribe( topicString )==1 ){
                Serial1.println( " Subscribed " );
                disablecallback = true;           // on subscribption do not process the message. Only want to process subsequent messages
                previousMillis4 = millis();  // Remember the time
         }
         
      }

      // Publish status information
      topicString ="channels/" + String( writechannelID ) + "/publish/"+String(writeAPIKey);
      data = String("field1=" + Pool_TEMP + "&field2=" + PM_TXT + "&field3=" + Pool_SPA + "&field4=" + Pool_PUMP);
      
      mqttPublish( topicString, data );
      
      //Serial.print("Temperature is : ");
      //Serial.print(Pool_TEMP);
      //Serial.print(" Mode: ");
      //Serial.print(PM_TXT);
      //Serial.print(" SPA: ");
      //Serial.print(Pool_SPA);
      //Serial.print(" PUMP: ");
      //Serial.println(Pool_PUMP);
    }
    // Reset disable after 10 seconds to process normal requests. The initial subscription message is ignored.
    if ((millis() - previousMillis4 >= disableInterval))  {
        disablecallback = false;
    }
    // Only enable MQTT loop if Alexa option is enabled
    if (enableAlexa == true) mqttClient.loop();
    delay(5);

    // WiFi / WebServer Functions which support the Phone Application
    // Check if a client has connected
    WiFiClient client = server.available();
    if (!client) {
      return;
    }
      
    // Wait until the client sends some data
    //Serial.println("new client");
    while(!client.available()){
      delay(1);
    }
  
    // Read the first line of the request
    String req = client.readStringUntil('\r');
    //Serial.println(req);
    client.flush();
  
    // Match the request
    // Pool Mode Change sets the PM_Button to 1
    // Other requests set the device on/off and set the Nextion picture to match.
    // All updates on the Nextion are reflected on the Phone App and vice versa.
    if (req.indexOf("/PM/Change") != -1) {
      //Serial.println("/PM/Change");
      au16data[PM_Button] = 1;
    }
    else if (req.indexOf("/SPA/ON") != -1) {
      //Serial.println("/SPA/ON");
      au16data[SPA_VLV] = 1;
      p0.setPic(2);
    }
    else if (req.indexOf("/SPA/OFF") != -1) {
      //Serial.println("/SPA/OFF");
      au16data[SPA_VLV] = 0;
      p0.setPic(1);
    }
    else if (req.indexOf("/LIGHT1/ON") != -1) {
      //Serial.println("/LIGHT1/ON");
      au16data[Light_1] = 1;
      p1.setPic(4);
    }
    else if (req.indexOf("/LIGHT1/OFF") != -1) {
      //Serial.println("/LIGHT1/OFF");
      au16data[Light_1] = 0;
      p1.setPic(3);
    }
    else if (req.indexOf("/LIGHT2/ON") != -1) {
      //Serial.println("/LIGHT2/ON");
      au16data[Light_2] = 1;
      p2.setPic(4);
    }
    else if (req.indexOf("/LIGHT2/OFF") != -1) {
      //Serial.println("/LIGHT2/OFF");
      au16data[Light_2] = 0;
      p2.setPic(3);
    }
    else if (req.indexOf("/Update") != -1) {
      //Serial.println("//Update");
    }
    else {
      //Serial.println("invalid request");
      client.stop();
      return;
    }

    client.flush();

    // Return the response

    String s = Pool_TEMP + ":" + PM_TXT + ":" + au16data[Pump] + ":" + au16data[SPA_VLV] + ":" + au16data[Light_1] + ":" + au16data[Light_2];
    //Serial.print("Client data: ");
    //Serial.println(s);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println(""); //  do not forget this one
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    //client.print("22.5:AUTO:1:1:0:1");  // Test Data to send to client Temperature: Pool Mode: Pump Status: SPA VLV Status: Light1 Status: Light2 Status
    client.print(s);  // Data to send to client Temperature: Pool Mode: Pump Status: SPA VLV Status: Light1 Status: Light2 Status
    client.println("</html>");
  
    delay(1);
    //Serial.println("Client disonnected");

    // The client will actually be disconnected 
    // when the function returns and 'client' object is destroyed
}

