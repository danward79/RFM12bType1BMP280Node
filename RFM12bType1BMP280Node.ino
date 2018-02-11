/* Sensor Transmitter - Type 1
	RFM12bType1BMP280Node

	Type 1 sensor provides the following data:
	sensor type (byte), light level 0-100% (byte), temperature C (int), pressure hPa (long) and VCC/battery voltage (byte)
	Both temperature and pressure should be scaled at the receiving end, by 10 and 100 respectively to give the correct decimal representation.

	The received packet takes the form of:
	struct {byte sensortype; byte light; int temperature; long pressure; byte vcc; } payload;

	The circuit: ATMega328/Arduino
	LDR - A0 to Gnd
	BMP280 - I2C A4/A5
*/

// Debug setting
#define DEBUG 0

// User settings
#define SEND_PERIOD 60000	// Minimum transmission period of sensor values
#define NODE_ID 16          		
#define GROUP 212  
#define NODE_LABEL "\n[Type 1]" 

// General parameters which may need to be changed
#define LDR_PORT 0   				// Defined if LDR is connected to a port's AIO pin
#define FREQ RF12_433MHZ        	// Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
#define SENSOR_TYPE 1
#define SERIAL_BAUD 9600

// Includes
#include <JeeLib.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

// General Variables
struct {byte sensortype; byte light; int temperature; long pressure; byte vcc; } payload;

volatile bool adcDone;
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
ISR(ADC_vect) { adcDone = true; }
// End of config

static byte readVcc (byte count =4) {
    set_sleep_mode(SLEEP_MODE_ADC);
    ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
    bitSet(ADCSRA, ADIE);
    while (count-- > 0) {
      adcDone = false;
      while (!adcDone)
        sleep_mode();
    }
    bitClear(ADCSRA, ADIE);  
    // convert ADC readings to fit in one byte, i.e. 20 mV steps:
    //  1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
	return (55U * 1023U) / (ADC + 1) - 50;
}

// Setup Routine
void setup() {   
#if DEBUG
	Serial.begin(SERIAL_BAUD);
	Serial.println(NODE_LABEL);
#endif
	
	rf12_initialize(NODE_ID, FREQ, GROUP);                    
	rf12_sleep(RF12_SLEEP);

	pinMode(14+LDR_PORT, INPUT);
	digitalWrite(14+LDR_PORT, 1); // pull-up
  
  if (!bmp.begin(0x76)) {  
#if DEBUG
    Serial.println("BMP280 sensor missing, check wiring!");
#endif
  }
}

// Main Loop
void loop() {
	//Temp & Pressure Sensor Readings
	payload.sensortype = (byte) SENSOR_TYPE;
	payload.light = 255 - analogRead(LDR_PORT) / 4;
	payload.temperature = (int) (bmp.readTemperature() * 10); 
	payload.pressure = (long) bmp.readPressure(); 
	payload.vcc = readVcc();
				
	//send sensor readings		
	send_rf_data();  
      
#if DEBUG
		printData();
#endif

	Sleepy::loseSomeTime(SEND_PERIOD);
}

//transmit payload
void send_rf_data()
{
  rf12_sleep(RF12_WAKEUP);
  // if ready to send + exit loop if it gets stuck as it seems too
  int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}
  rf12_sendStart(0, &payload, sizeof payload);//RF12_HDR_ACK
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);

  rf12_sleep(RF12_SLEEP);  
}

//debug print - set debug to 1
void printData() {
	Serial.print(" t "); Serial.println(payload.sensortype, DEC);
	Serial.print(" l "); Serial.println(payload.light, DEC);
	Serial.print(" t "); Serial.println(payload.temperature, DEC);
	Serial.print(" p "); Serial.println(payload.pressure, DEC);
	Serial.print(" v "); Serial.println(payload.vcc, DEC);
}