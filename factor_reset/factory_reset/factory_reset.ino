/*
 * Attempt to reset the bluefruit module. When the module is not responding to any commands
 * None of the changes in this code seem to make any difference.
 * The only thing that seems to help is to connect the reset pad under the module to ground
 * when the board is powered up.
 */

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEGatt.h"
#include "Adafruit_BLEBattery.h"

#include "BluefruitConfig.h"

/* Create the bluefruit object, if you want to use this sketch with a different bluefruit
 *  module configuration see the bluefruit examples for the different configurations.
 *  This is specific to the bluefruit feather:
 *  hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
 */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(50);                       // wait for a 0.1 seconds
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(50);                       // wait for a 0.1 seconds
  }
}

void atzReset() {
  // Try a basic reset
  if( ble.reset(true) ) {
    // If it succeeds then we are all set
    Serial.println( F("ATZ reset worked. Stopping"));
    while(1);    
  }

  // When ATZ fails then there is typically a '<-' left on the line
  Serial.println();
  Serial.println( F("ATZ reset failed."));  
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output and turn off the light for 0.5 s
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  digitalWrite(LED_BUILTIN, HIGH);

  while (!Serial);  // wait for console, only run this while device is attached
  // put your setup code here, to run once:

  Serial.begin(115200);

  Serial.println( F("Starting Sketch") );

  Serial.print( F("Calling ble.begin(), result: ") );
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK") );

  atzReset();

  // Test out a new and improved NVMRead
  int32_t magic_number;
  if( ble.readNVM(0, &magic_number) ){
    error(F("Successfully read NVM when really there was no response"));
  }

  // My experience is that if a soft reset doesn't work then a software based
  // hard reset doesn't work either. But it can't hurt to try
  if ( ble.factoryReset() ){
    // If it succeeds then we are all set
    Serial.println( F("Software factory reset worked. Stopping"));
    while(1);        
  }

  // When factoryReset fails then there is typically a '<-' left on the line
  Serial.println();
  Serial.println( F("Software factory reset failed."));

  // Try doing a hardware factory reset

  Serial.println( F("Trying a reset like what ble.begin does"));

  // This is just a reset, but not a factory reset, it was taken from the
  // the ble.begin code, and hte dely was increased to 1000 ms
  // When the module was not responding to any commands this did not actually help
  pinMode(BLUEFRUIT_SPI_RST, OUTPUT);
  digitalWrite(BLUEFRUIT_SPI_RST, HIGH);
  digitalWrite(BLUEFRUIT_SPI_RST, LOW);
  delay(1000);
  digitalWrite(BLUEFRUIT_SPI_RST, HIGH);

  Serial.println( F("Checking if module is back online"));
  atzReset();

  error(F("Cannot reset bluefruit. Probably there is something wrong with the board"));
}

void loop() {
  // put your main code here, to run repeatedly:

}
