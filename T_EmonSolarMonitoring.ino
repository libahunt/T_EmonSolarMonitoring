/*

 
 emonTx documentation: 	http://openenergymonitor.org/emon/modules/emontxshield/
 emonTx firmware code explination: http://openenergymonitor.org/emon/modules/emontx/firmware
 emonTx calibration instructions: http://openenergymonitor.org/emon/modules/emontx/firmware/calibration

 THIS SKETCH REQUIRES:

 Libraries in the standard arduino libraries folder:
	- JeeLib		https://github.com/jcw/jeelib
	- EmonLib		https://github.com/openenergymonitor/EmonLib.git

 Other files in project directory (should appear in the arduino tabs above)
	- emontx_lib.ino
 
*/

/*Recommended node ID allocation
------------------------------------------------------------------------------------------------------------
-ID-	-Node Type- 
0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
1-4     - Control nodes 
5-10	- Energy monitoring nodes
11-14	--Un-assigned --
15-16	- Base Station & logging nodes
17-30	- Environmental sensing nodes (temperature humidity etc.)
31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
-------------------------------------------------------------------------------------------------------------
*/
#include <Wire.h>

#define FILTERSETTLETIME 5000                                           //  Time (ms) to allow the filters to settle before sending data

const int CT1 = 1; 
const int CT2 = 1;                                                      // Set to 0 to disable 
const int CT3 = 1;
//const int CT4 = 1; //CT4 is disabled

#define RF_freq RF12_433MHZ                                             // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 13;                                                  // emonTx RFM12B node ID
const int networkGroup = 210;                                           // emonTx RFM12B wireless network group - needs to be same as emonBase and emonGLCD                                                 

#define RF69_COMPAT 0 // set to 1 to use RFM69CW 
#include <JeeLib.h>   // make sure V12 (latest) is used if using RFM69CW
#include "EmonLib.h"
EnergyMonitor ct1,ct2,ct3,ct4;                                           // Create  instances for each CT channel

typedef struct { int power1, power2, power3, Vrms, solarA, solarBatV, solarPanelV; } PayloadTX;      // create structure - a neat way of packaging data for RF comms
PayloadTX emontx;                                                       

const int LEDpin = 9;                                                   // On-board emonTx LED 

boolean settled = false;

/* variables for solar panel monitoring */
int batVPin = A4;    // voltage divider of battery voltage
int panelVPin = A0;  // voltage divider of solar panel difference voltage
int ampPin = A5;     // ampmeter module on solar charging wire
float supplyV = 5.0; // will get a preciser value from internal voltage reference in each loop

//battery voltage divider
int batR1 = 9910; // Resistance of R1 in ohms
int batR2 = 3274; // Resistance of R2 in ohms
float batRRatio = (float)batR1 / (float)batR2;

/* ... dont understand the previous thing, should be:
equation: batteryVoltage = pinVoltage*batR2/(batR1+batR2); ??
*/


//panel voltage divider
int panR1 = 3603;
int panR2 = 2447;
int panR3 = 1980;
/* The equation is panelVoltage = (pinVoltage*(panR2+panR1+panR1*panR2/panR3) - panR1*supplyV)/panR2 */
float panelHelperRRatio = (float)panR2 + (float)panR1 + (float)panR1*(float)panR2/(float)panR3;
/* Now simplified equation to be used: panelVoltage = (pinVoltage*panelHelperRRatio - panR1*supplyV)/panR2 */

float amps = 0;
int i; 
int solarAmpReadings[50];
int avgSolarAmpReading;

int sampleVal = 0;
int avgVal = 0;       
float pinVoltage = 0;

float batteryVoltage = 0;
float panelVoltage = 0;

float batteryVoltageAlternate = 0;//for testing
float panelVoltageAlternate = 0;//for testing


int sensorValue = 0;        // value read from the carrier board
int outputValue = 0;        // output in milliamps

/*int solaramps = 0;
int solarvolts = 0;
int solarPanelv = 0;*/
// volt end




void setup() 
{
  Serial.begin(9600);
   //while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  
  Serial.println("emonTX Shield CT123 Voltage example"); 
  Serial.println("OpenEnergyMonitor.org");
  Serial.print("Node: ");
  Serial.print(nodeID);
  Serial.print(" Freq: ");
  if (RF_freq == RF12_433MHZ) Serial.print("433Mhz");
  if (RF_freq == RF12_868MHZ) Serial.print("868Mhz");
  if (RF_freq == RF12_915MHZ) Serial.print("915Mhz"); 
  Serial.print(" Network: "); 
  Serial.println(networkGroup);
  // }
   
  if (CT1) ct1.current(1, 60.606);                                     // Setup emonTX CT channel (ADC input, calibration)
  if (CT2) ct2.current(2, 60.606);                                     // Calibration factor = CT ratio / burden resistance
  if (CT3) ct3.current(3, 60.606);                                     // emonTx Shield Calibration factor = (100A / 0.05A) / 33 Ohms
//  if (CT4) ct4.current(4, 60.606); 
  
  if (CT1) ct1.voltage(0, 268.97, 1.7);                                // ct.voltageTX(ADC input, calibration, phase_shift) - make sure to select correct calibration for AC-AC adapter  http://openenergymonitor.org/emon/modules/emontx/firmware/calibration. Default set for Ideal Power adapter                                         
  if (CT2) ct2.voltage(0, 268.97, 1.7);                                // 268.97 for the UK adapter, 260 for the Euro and 130 for the US.
  if (CT3) ct3.voltage(0, 268.97, 1.7);
//  if (CT4) ct4.voltage(0, 268.97, 1.7);
  
  rf12_initialize(nodeID, RF_freq, networkGroup);                          // initialize RFM12B
  rf12_sleep(RF12_SLEEP);

  pinMode(LEDpin, OUTPUT);                                              // Setup indicator LED
  digitalWrite(LEDpin, HIGH);
  
                                                                                     
}

void loop() 
{ 
  
//supply voltage measurement for more precise analogRead processing
  supplyV = readVcc()*1000;
  
  
//battery voltage measurement
  sampleVal = 0;
  avgVal = 0;
  for (int x = 0; x < 10; x++) {
    sampleVal = sampleVal + analogRead(batVPin);
    delay(10);
  }
  avgVal = sampleVal / 10;

  pinVoltage = avgVal * 0.006535; 
  batteryVoltage = pinVoltage * batRRatio;

  pinVoltage = avgVal * supplyV/1023;//for testing
  batteryVoltageAlternate = pinVoltage*batR2/(batR1+batR2);//for testing


//panel voltage measurement
  sampleVal = 0;
  avgVal = 0;
  for (int x = 0; x < 10; x++) {
    sampleVal = sampleVal + analogRead(panelVPin);
    delay(10);
  }
  avgVal = sampleVal / 10;

  pinVoltage = avgVal * supplyV/1023; 
  panelVoltage = batteryVoltage - (pinVoltage*panelHelperRRatio - panR1*supplyV)/panR2;
  panelVoltageAlternate = batteryVoltageAlternate - (pinVoltage*panelHelperRRatio - panR1*supplyV)/panR2;//for testing


//solar amperage measurement       
  for (i=0; i<50;i++){
    solarAmpReadings[i]=analogRead(ampPin);
    delay(10);
  }
  avgSolarAmpReading = 0;
  for (i=0; i<50;i++){
    avgSolarAmpReading = avgSolarAmpReading + solarAmpReadings[i];
  }
  avgSolarAmpReading = avgSolarAmpReading/50;
  
  amps = (512 - avgSolarAmpReading) * 27.03 / 1023;
  
  
//create data package as integer numbers
  emontx.solarA = amps*1000;
  emontx.solarBatV = batteryVoltage*100;
  emontx.solarPanelV = panelVoltage*100;
  
//serial
  Serial.print("Battery voltage:  " );                 
  Serial.print(batteryVoltage); 
  Serial.print("  Panel voltage: "); 
  Serial.print(panelVoltage);
  Serial.print("  Amps: "); 
  Serial.println(amps);
  Serial.print("Battery v altern: " );                 
  Serial.print(batteryVoltageAlternate); 
  Serial.print("  Panel v alter: "); 
  Serial.println(panelVoltageAlternate);
  
  
  
  /* if (CT1) {
  ct1.calcVI(20,2000);                                                  // Calculate all. No.of crossings, time-out 
  emontx.power1 = ct1.realPower;
  Serial.print(emontx.power1);                                         
  }
  
  emontx.Vrms = ct1.Vrms*100;                                            // AC Mains rms voltage 
  
  
  if (CT2) {
  ct2.calcVI(20,2000);                                                  // Calculate all. No.of crossings, time-out 
  emontx.power2 = ct2.realPower;
  Serial.print(" "); Serial.print(emontx.power2);
  } 
  
  if (CT3) {
  ct3.calcVI(20,2000);                                                  // Calculate all. No.of crossings, time-out 
  emontx.power3 = ct3.realPower;
  Serial.print(" "); Serial.print(emontx.power3);
  } 
  
  if (CT4) {
   ct4.calcVI(20,2000);                                                  // Calculate all. No.of crossings, time-out 
  emontx.power4 = ct4.realPower;
  Serial.print(" "); Serial.print(emontx.power4);
  } 
  
  Serial.print(" "); Serial.print(ct1.Vrms);
  
  
  */ 
  Serial.println(); delay(100);
  
  
  
  // because millis() returns to zero after 50 days ! 
  if (!settled && millis() > FILTERSETTLETIME) settled = true;
  
  if (settled)                                                            // send data only after filters have settled
  { 
    send_rf_data();                                                       // *SEND RF DATA* - see emontx_lib
    digitalWrite(LEDpin, HIGH); delay(2); digitalWrite(LEDpin, LOW);      // flash LED
    delay(2000);                                                          // delay between readings in ms
  }
}



//Precise Vcc value for better analogRead interpretation
//from http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc() { 
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

