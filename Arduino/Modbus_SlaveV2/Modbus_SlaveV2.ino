 /**
 *  Modbus slave for arduino uno
 * pin maping:
 * 0 - RX
 * 1 - TX
 * 2 - TxEnable
 * 3 - digital output
 * 4 - digital output
 * 5 - digital output
 * 6 - digital output
 * 7 - digital output
 * 8 - digital input
 * 9 - digital input
 * 10 - digital input
 * 11 - digital input
 * 12 - analog input - Datawire type temperature
 * 13 - analog input - Datawire type temperature
 * 
 * A0 - analog input - Thermistor type temperature
 * A1 - analog input - Thermistor type temperature
 */

#include <ModbusRtu.h>
#define ID   1                // Modbus Slave Address
#define SERIALPORT   0        // serial port: 0-Serial, 1..3-Serial1..Serial3; 4: use software serial
#define TxEnable   0          // flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
#define BaudRate   9600       // set serial baudrate
#define Parity   SERIAL_8N1   // set serial parity
#define Version   10          // Version Number *10, 10 = Version 1.0

  //  Setup using Thermistor type temperature on Arduino analog inputs
#include <math.h>         // loads the more advanced math functions - comment out if using Data Wire
#define filter  0.01        // set filter weight >0 and <=1. This sets the filter for the temperature sensor 0.5 is average

  // Setup using Data wire is plugged into pin 12,13 on the Arduino for temperature inputs
//#include <DallasTemperature.h>    // Comment out if using Thermistor
//#include <OneWire.h>
//#define ONE_WIRE_BUSA 12  // Temperature 1 AI
//#define ONE_WIRE_BUSB 13  // Temperature 2 AI

  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire oneWireA(ONE_WIRE_BUSA);
//OneWire oneWireB(ONE_WIRE_BUSB);
  // Pass our oneWire reference to Dallas Temperature. 
//DallasTemperature sensorsA(&oneWireA);
//DallasTemperature sensorsB(&oneWireB);

// Create new modbus instance
Modbus slave(ID, SERIALPORT, TxEnable); // this is slave ID and RS-232 or USB-FTDI

// Slave registers: Note Digital IN/OUT have to be the first registers
enum {
  Digital_IN,       // Register 0 PM_OFF Bit 0, PM_ON Bit 1, PUMP Bit 2
  Digital_OUT,      // Register 1 Not used - using registers for DO instead of coils.
  PM_CTRL,          // Register 2 Pool Mode DO
  SPA_VLV,          // Register 3 Pool/Spa Valve DO
  Spare_VLV,        // Register 4 Spare Valve DO
  Light_1,          // Register 5 Light 1 DO
  Light_2,          // Register 6 Light 2 DO
  PM_Button,        // Register 7 Pool Mode change button
  TIMESCH1,         // Register 8 Time Schedule 1 value
  TIMESCH2,         // Register 9 Time Schedule 2 value
  TIMESCH3,         // Register 10 Time Schedule 3 value
  TIMESCH4,         // Register 11 Time Schedule 4 value
  TIMESCH5,         // Register 12 Time Schedule 5 value  
  Temp_1,           // Register 13 Temperature 1 AI
  Temp_2,           // Register 14 Temperature 2 AI
  Pool_Mode,        // Register 15 Pool Mode Value 0=Auto, 1=ON, 2=OFF, 3=ERROR
  Pump,             // Register 16 Pump Running 0=OFF, 1=ON
  MSG_IN,           // Register 17 Message IN Count
  MSG_OUT,          // Register 18 Message OUT Count
  MSG_ERR,          // Register 19 Message ERROR Count
  Display_Baud,     // Register 20 Device Baud Rate
  Display_ID,       // Register 21 Modubus Slave ID
  Display_Version,  // Register 22 Software Version #
  Slave_REGS        // Dummy register. using 0 offset to keep size of array
};

// data array for modbus network sharing
uint16_t au16data[Slave_REGS];

unsigned long previousMillis = 0;        // will store last time LED was updated
long OnTime = 250;           // milliseconds of on-time
long OffTime = 250;          // milliseconds of off-time
unsigned long previousMillis2 = 0;        // will store last time temperature was updated
long delayTime = 1000;           // milliseconds of delay-time for temperature refresh
int averagetemp1 = 0;        // average temp1 reading
//int averagetemp2 = 0;        // average temp2 reading

/**
 *  Setup procedure
 */
void setup() {
    // define i/o
  pinMode(TxEnable,OUTPUT);      //   TxEnable Control for RS485  
  pinMode(3,OUTPUT);      //   PM_CTRL - Pool Mode DO
  pinMode(4,OUTPUT);      //   SPA_VLV - Pool/Spa Valve DO
  pinMode(5,OUTPUT);      //   Spare_VLV - Spare Valve DO
  pinMode(6,OUTPUT);      //   Light_1 - Light_1 DO
  pinMode(7,OUTPUT);      //   Light_2 - Light_2 DO
  pinMode(8,INPUT_PULLUP);  //  PM_OFF - DI
  pinMode(9,INPUT_PULLUP);  //  PM_ON - DI
  pinMode(10,INPUT_PULLUP); //  PUMP - DI
  pinMode(11,INPUT_PULLUP); //  Not used - DI

 // start communication
  //Serial.begin(9600); //Begin serial communication
  slave.begin( BaudRate, Parity );  // set slave baudrate, parity
  averagetemp1 = analogRead(0);     // set initial value for first analog read of sensor
  //averagetemp2 = analogRead(1);
}

  //  Setup using Thermistor type temperature on Arduino analog inputs - comment out if using Data Wire
  //Function to perform the fancy math of the Steinhart-Hart equation
  // RawADC is the raw analogread() value.
  // nomRes is the thermistor nominal resistance, usually 10000
  // bCoef is the thermistor beta coefficient, usually 3950 for a 10K thermistor
  // serialRes is the actual series resistor value, nominally 10000
float Thermistor (uint16_t RawADC, uint16_t nomRes, uint16_t bCoef, uint16_t serialRes) {
#define ADC_RESOLUTION 1023
#define TEMPERATURENOMINAL 25
  float tempval = 0;

  tempval = RawADC;
  //Serial.print("Raw analog reading "); 
  //Serial.println(RawADC);

  // convert the value to resistance
  tempval = ADC_RESOLUTION / tempval - 1;
  tempval = serialRes * tempval;
  //Serial.print("Thermistor resistance "); 
  //Serial.println(tempval);
 
  float steinhart;
  steinhart = tempval / nomRes;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= bCoef;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  //Serial.print("Temperature "); 
  //Serial.print(steinhart);
  //Serial.println(" *C");
   
  return steinhart;
}
/**
 *  Loop procedure
 */
void loop() {
  // poll messages
  slave.poll( au16data, Slave_REGS );

  // link the Arduino pins to the Modbus array
  // get digital inputs -> au16data[Digital_IN], bit number
                                                          //    MODBUS ADDRESS
  bitWrite( au16data[Digital_IN], 0, digitalRead( 8 ));   //    10001   PM_OFF
  bitWrite( au16data[Digital_IN], 1, digitalRead( 9 ));   //    10002   PM_ON
  bitWrite( au16data[Digital_IN], 2, digitalRead( 10 ));  //    10003   PUMP
  bitWrite( au16data[Digital_IN], 3, digitalRead( 11 ));  //    10004   Not used


  // set digital outputs -> 
  // PM_CTRL is a momentary output controlled by PM_Button. The value of PM_Button sets the number of pulses
  
    if((au16data[PM_CTRL] == HIGH) && (millis() - previousMillis >= OnTime))
  {
    au16data[PM_CTRL] = LOW;  // Turn it off
    previousMillis = millis();  // Remember the time
    digitalWrite(3, au16data[PM_CTRL]);  // Update the actual LED 40003
    au16data[PM_Button]--;
  }
  else if ((au16data[PM_CTRL] == LOW) && (millis() - previousMillis >= OffTime) && (au16data[PM_Button] > 0))
  {
    au16data[PM_CTRL] = HIGH;  // turn it on
    previousMillis = millis();   // Remember the time
    digitalWrite(3, au16data[PM_CTRL]);    // Update the actual LED 40003
  }

  // These DO are controlled directly by the Modbus register
  digitalWrite( 4, au16data[SPA_VLV]);                          //    40004
  digitalWrite( 5, au16data[Spare_VLV]);                        //    40005
  digitalWrite( 6, (au16data[Light_1] || au16data[TIMESCH1]));  //    40006 Turn ON with light switch OR time schedule
  digitalWrite( 7, (au16data[Light_2] || au16data[TIMESCH2]));  //    40007
                                                                //    40008 This is PM_Button Register
 
  if((millis() - previousMillis2 >= delayTime)) {
  
      previousMillis2 = millis();  // Remember the time    
      // read Thermistor type analog inputs - comment out if using Data Wire
      averagetemp1 = int (filter * analogRead(0) + (1 - filter) * averagetemp1);      // filter the raw ADC value to reduce fluctuations
      au16data[Temp_1] = int (Thermistor(averagetemp1,10000,3950,10000) * 10);        //    30014
      //Serial.print("Temperature is: "); 
      //Serial.println(au16data[Temp_1]);
  
                                                                                  // Thermistor value is *10 and coverted to integer for modbus
      //averagetemp2 = int (filter * analogRead(1) + (1 - filter) * averagetemp2);
      //au16data[Temp_2] = int (Thermistor(averagetemp2,10000,3950,10000) * 10);      //    30015
  } 
      // read Datawire type analog inputs - comment out if using Thermistor
  //sensorsA.requestTemperatures();
  //sensorsB.requestTemperatures();
  //au16data[Temp_1] = (int) (sensorsA.getTempCByIndex(0) * 10);  //  30014
  //au16data[Temp_2] = (int) (sensorsB.getTempCByIndex(0) * 10);  //  30015

  if (!digitalRead( 8 ) && !digitalRead( 9 )) au16data[Pool_Mode] = 0;  // Set Pool_Mode based on PM_OFF, PM_ON status
  if (!digitalRead( 8 ) && digitalRead( 9 )) au16data[Pool_Mode] = 1;  // 30016
  if (digitalRead( 8 ) && !digitalRead( 9 )) au16data[Pool_Mode] = 2;
  if (digitalRead( 8 ) && digitalRead( 9 )) au16data[Pool_Mode] = 3;

  if (digitalRead( 10 )) au16data[Pump] = 1;   //  Pump ON    30017
  if (!digitalRead( 10 )) au16data[Pump] = 0;  //  Pump OFF    
  
  // diagnose communication
  au16data[MSG_IN] = slave.getInCnt();                    //    30018
  au16data[MSG_OUT] = slave.getOutCnt();                  //    30019
  au16data[MSG_ERR] = slave.getErrCnt();                  //    30020
  au16data[Display_Baud] = BaudRate;                      //    30021
  au16data[Display_ID] = ID;                              //    30022
  au16data[Display_Version] = Version;                    //    30023
} 

