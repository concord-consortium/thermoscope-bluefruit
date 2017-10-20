/*
 * Need to figure out the licensing of the Adafruit code.
It says MIT but then it adds conditions: like including all of the text above and the splash screen.
The thermistor code was taking from the Adafruit learn page.
The bluetooth code was copied from their healththermometer example.
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

// which analog pin to connect A sensor
#define THERMISTORPINA A2
// which analog pin to connect B sensor
#define THERMISTORPINB A5

// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 80

// Battery pin
#define VBATPIN A7

// MAGIC Byte to check if non volitale memory was write by us
#define MAGIC_NUMBER 0x03CDADDA

// NVM version form, this could be used to migrate older devices and preserve their data
#define NVM_VERSION 1

#include "thermoscope.h"

int samples[NUMSAMPLES];

/* Create the bluefruit object, if you want to use this sketch with a different bluefruit
 *  module configuration see the bluefruit examples for the different configurations.
 *  This is specific to the bluefruit feather:
 *  hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
 */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Setup the gatt object
Adafruit_BLEGatt gatt(ble);

// Setup battery object
Adafruit_BLEBattery battery(ble);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


/* The service information */
int32_t iconCharCharacteristicId;

SensorServiceInfo sensorServiceA;
SensorServiceInfo sensorServiceB;

uint32_t disconnectedCount;

// This initialized the struct so the first time it is written the defaults
// get written too
static UserConfigV1 userConfigV1 = {
  {0}, // nul iconChar
  {
    10000, // seriesResistance
    10000, // thermistorNominalResistance
    3950   // thermistorCoefficient
  },
  {
    10000, // seriesResistance
    10000, // thermistorNominalResistance
    3950  // thermistorCoefficient
  }
};

GattCallback callbackMap[30];

void addGattRxCallback(int32_t id, GattCallback gattCallback) {
  callbackMap[id] = gattCallback;
  ble.setBleGattRxCallback(id, BleGattRX);
}

void updateNVM()
{
  ble.writeNVM(8, (uint8_t *)&userConfigV1, sizeof(UserConfigV1));
}

void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
  if(chars_id >= 30) {
    return;
  }
  GattCallback callback = callbackMap[chars_id];
  if(callback == NULL){
    return;
  }

  callback(chars_id, data, len);
}

/*
  Handle an incoming id string. This is string is saved in
  the bluefruits non volitale memory, and it used as a suffixed to
  "Thermoscope " to make it easier to identify the device.
*/
void BleGattRXId(int32_t chars_id, uint8_t data[], uint16_t len)
{
  Serial.print( F("[BLE GATT RX] id:" ) );
  Serial.print(chars_id);
  Serial.print(F(", len: "));
  Serial.print(len);
  Serial.print(F(" "));
  
  if (chars_id == iconCharCharacteristicId)
  {   
    Serial.write(data, len);
    Serial.println();
    // save this into NVM

    // the incoming data buffer is not null terminated.
    // set the string to be all 0s before copying in new characters
    memset(userConfigV1.iconChar, 0, ICON_CHAR_LEN);

    // leave at least one blank byte at the end of the iconChar this way at least
    // one of the 0s that we added before will be at the end of the string.
    // this way it should guaruntee the string is NUL terminated
    strncpy(userConfigV1.iconChar, (char *)data, min(len,(ICON_CHAR_LEN - 1)));
    updateNVM();

    // it would be good to also change the name of the device here
    // but in the meantime the user can just reset after doing this
    setDeviceName(userConfigV1.iconChar);

    // reset the device after setting its name
    ble.reset();
  }
}

void BleGattRXSensor(SensorServiceInfo *sensorService, TempCoefficients *coefficients,
                     int32_t chars_id, uint8_t data[], uint16_t len)
{
  Serial.print( F("[BLE GATT RX] id:" ) );
  Serial.print(chars_id);
  Serial.print(F(", val: "));
  // all the data should be 16 bit unsigned integers
  uint16_t value;
  memcpy(&value, data, 2);
  Serial.println(value);

  if (chars_id == sensorService->seriesResistanceCharId) {
    coefficients->seriesResistance = value;
  } else if (chars_id == sensorService->thermistorNominalResistanceCharId)  {
    coefficients->thermistorNominalResistance = value;
  } else if (chars_id == sensorService->thermistorBetaCharId) {
    coefficients->thermistorBeta = value;
  }
  updateNVM();
}

void BleGattRXSensorA(int32_t chars_id, uint8_t data[], uint16_t len)
{
  BleGattRXSensor(&sensorServiceA, &userConfigV1.sensorA, chars_id, data, len);
}

void BleGattRXSensorB(int32_t chars_id, uint8_t data[], uint16_t len)
{
  BleGattRXSensor(&sensorServiceB, &userConfigV1.sensorB, chars_id, data, len);
}

int32_t addIntegerOutputCharacteristic(uint16_t uuid) {
  int32_t id = gatt.addCharacteristic(
    uuid,
    GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY, // read and notify
    4, // min len
    4, // max len
    BLE_DATATYPE_BYTEARRAY); // tried using integer here but underlying gatt library always
                             // uses bytearray which then caused an error when calling writeChar

  if (id == 0) {
    error(F("Could not add characteristic"));
  }

  return id;
}


int32_t addCoefficientCharacteristic(uint16_t uuid, uint16_t value, GattCallback gattCallback) {
  int32_t id;
  id = gatt.addCharacteristic(
    uuid,
    GATT_CHARS_PROPERTIES_WRITE | GATT_CHARS_PROPERTIES_READ,
    2, // min len
    2, // max len
    BLE_DATATYPE_BYTEARRAY); // tried using integer here but underlying gatt library always
                             // uses bytearray which then caused an error when calling writeChar

  if (id == 0) {
    error(F("Could not add characteristic"));
  }

  gatt.setChar(id, (uint8_t *)&value, 2);

  // add a callback so we get notified when this changes
  addGattRxCallback(id, gattCallback);

  return id;
}

void addSensorService(uint8_t serviceUUID[], SensorServiceInfo *info, TempCoefficients *coefficients,
  GattCallback gattCallback){
  /* Service ID should be 1 */
  // TODO see if we avoid this duplicate UUID def to prevent a mistake
  // The a normal uuid was generated and then the 3 and 4 bytes were 00'd out
  Serial.println(F("Adding Sensor Service"));
  info->serviceId = gatt.addService(serviceUUID);
  if (info->serviceId == 0) {
    error(F("Could not add service"));
  }

  // Add the Temperature Measurement characteristic which is an int with the temp in C times 100
  // so a temperatue of 30.01 which will be sent as 3001
  Serial.println(F("Adding the Temperature Measurement characteristic (UUID = 0x0001):"));
  info->measureCharId = addIntegerOutputCharacteristic(0x0001);

  // Add the microvolts characteristic which is an int
  // this won't be totally accurate micovolts because the voltage reference is not perfectly accurate
  // And this value is not used directly in the calculation.
  // The value used in the calibration is the voltage fraction or microVolts / 3,300,000
  Serial.println(F("Adding the Voltage Measurement characteristic (UUID = 0x0002):"));
  info->microVoltsCharId = addIntegerOutputCharacteristic(0x0002);

  // Add Series resistor characteristic an int with the resistance in Ohms.
  Serial.println(F("Adding the Series characteristic (UUID = 0x0003):"));
  info->seriesResistanceCharId =
    addCoefficientCharacteristic(0x0003, coefficients->seriesResistance, gattCallback);

  // Add thermistor nominal resistance characteristic an int with the resistance in Ohms.
  Serial.println(F("Adding the ThermistorNominalResistance characteristic (UUID = 0x0004):"));
  info->thermistorNominalResistanceCharId =
    addCoefficientCharacteristic(0x0004, coefficients->thermistorNominalResistance, gattCallback);

  // Add thermistor beta characteristic an int.
  Serial.println(F("Adding the ThermistorBeta characteristic (UUID = 0x0005):"));
  info->thermistorBetaCharId =
    addCoefficientCharacteristic(0x0005, coefficients->thermistorBeta, gattCallback);
}

void setDeviceName(char *iconChar) {
  // leave room for a [space] + 4 byte char + nul
  char device_name[17];
  strcpy_P(device_name, PSTR("Thermoscope"));

  if (strlen(iconChar) > 0) {
    // make sure we don't overflow the buffer incase the string is not null terminated
    size_t _iconCharLength = strnlen(iconChar, 4);
    
    Serial.print( F("Length of id char: "));
    Serial.println(_iconCharLength);
    Serial.println(iconChar);
    if(_iconCharLength > 0){
      strcat_P(device_name, PSTR(" "));
      // we've made sure iconCharLength is not greater than 4 so this way we shouldn't overflow
      // the device_name buffer
      strncat(device_name, iconChar, _iconCharLength);
    }
  }

  /* Change the device name to make it easier to find */
  Serial.print(F("Setting device name to '"));
  Serial.print(device_name);
  Serial.print(F("': "));

  if (! ble.atcommand(F("AT+GAPDEVNAME"), device_name)) {
    error(F("Could not set device name?"));
  }

}

void setup(void) {
  // while (!Serial);  // wait for console do not leave this in production

  // Not sure if this is needed or not
  delay(500);

  Serial.begin(115200);

  // would be good to have a version string here that was also published through gatt
  Serial.println(F("Bluefruit Thermoscope"));
  Serial.println(F("--------------------------------------------"));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  // Disable this so we can use the non-volitale memory
  //  Serial.println(F("Performing a factory reset: "));
  //  if (! ble.factoryReset() ){
  //       error(F("Couldn't factory reset"));
  //  }

  // ble.writeNVM(offset, str)
  // need to look at 'strlen' for the length of the string
  // the read call needs to pass the length of the string but it does do a copy operation that might stop
  // with the null characters so if the string is variable lenght
  // then I need to save this length
  // Need to figure out how to create a characteristic that can written to
  // We need these for the identifier and the calibration

//  ble.atcommand(F("AT+DBGNVMRD"));
  uint8_t data[256];
  // there is a max size of 64 bytes when reading like this
  // We are having an issue where the NVM data is cleared, it is not clear why yet.
  // One time while testing I saw this return all 0s, when it should have had data.
  // So it implies something is reseting it.
  // One fix would be to have a special sketch that initializes it, and then never reset it.
  // if the magic number doesn't match then the main sketch would just stop and flash the led.
  // this might help us narrow down the problem.
  ble.readNVM(0, data, 32);
  Serial.println("Current NVM");
  for(int i=0; i<16; i++){
    Serial.print(data[i], HEX);
    Serial.print(F(" "));
  }
  Serial.println(F(""));

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  int32_t magic_number;
  ble.readNVM(0, &magic_number);

  if ( magic_number != MAGIC_NUMBER )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Magic not found: performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }

    // Write data to NVM
    Serial.println( F("Write defined data to NVM") );
    ble.writeNVM(0, MAGIC_NUMBER);
    // Write NVM version
    ble.writeNVM(4, NVM_VERSION);
    // write the defaults
    updateNVM();
  }else
  {
    Serial.println(F("Magic found"));
    int32_t nvmVersion;
    ble.readNVM(4, &nvmVersion);
    Serial.print( F("NVM Version: ") );
    Serial.println(nvmVersion);

    ble.readNVM(8, (uint8_t *)&userConfigV1, sizeof(UserConfigV1));
  }

  setDeviceName(userConfigV1.iconChar);

  // TODO change the bluetooth level settings
  // set the power level: AT+BLEPOWERLEVEL
  if (! ble.atcommand(F("AT+BLEPOWERLEVEL=4"))) {
    error(F("Could not increase power level"));
  }
  
  // set the intervals: AT+GAPINTERVALS=[min connection int],[max conn int],[fast adv int],[fast adv timeout],[low pwr adv int]
  if (! ble.atcommand(F("AT+GAPINTERVALS=20,40,20,30,40"))) {
    error(F("Could not increase power level"));
  }
  

  // clear all services and characteristics
  // I can't tell if the values of characteristics is saved in NVM, if it was we could improve this
  // by using that to store all of the values, but then we'd need a way to clear selective services
  // incase we ever got rid of any
  gatt.clear();

  // add generic thermoscope service
  // add writable characteristic for the identifier
  // TODO add readable characteristic with a version
  gatt.addService(0x1234);

  // The AT command has the ability to set a default value but this helper function doesn't
  iconCharCharacteristicId = gatt.addCharacteristic(0x2345, GATT_CHARS_PROPERTIES_WRITE | GATT_CHARS_PROPERTIES_READ, 
    1, 5, BLE_DATATYPE_STRING, "identifier char");

  // set the default value of the iconChar
  // this assumes the string is always null terminated. That should be the case if the userConfig
  // was initialized correctly.
  // Note: the iconChar is a char[].  If a its type was a uint8_t[] then the setChar method trys to do
  // a conversion so if the iconChar was intended to be 'h', the actual value saved would be '68'
  gatt.setChar(iconCharCharacteristicId, userConfigV1.iconChar);

  // add a callback so we get notified when this changes
  addGattRxCallback(iconCharCharacteristicId, BleGattRXId);

  // TODO: Using the autogenerated one below didn't work for some reason???
  // {0xbc, 0x44, 0x00, 0x00, 0xf2, 0x44, 0x4f, 0x49, 0xb6, 0x56, 0xdf, 0x49, 0x85, 0x77, 0x55, 0xe4 };
  // So instead we are using some other vendor UUIDs that I know work
  uint8_t sensorAServiceUUID[] =
    {0xf0, 0x00, 0xaa, 0x00, 0x04, 0x51, 0x40, 0x00, 0xb0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  addSensorService(sensorAServiceUUID, &sensorServiceA, &userConfigV1.sensorA, BleGattRXSensorA);

  uint8_t sensorBServiceUUID[] =
    {0xf0, 0x00, 0xbb, 0x00, 0x04, 0x51, 0x40, 0x00, 0xb0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  addSensorService(sensorBServiceUUID, &sensorServiceB, &userConfigV1.sensorB, BleGattRXSensorB);

  // Switch to 12 bit resolution
  analogReadResolution(12);

  // Enable Battery service do not reset, we'll do that ourselves
  battery.begin(false);

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();

  disconnectedCount = 0;
}

void readTemperature(SensorServiceInfo *sensorService, const __FlashStringHelper*label, int32_t pin, TempCoefficients *coefficients){
  uint8_t i;
  float averageDigitialCounts;

  // take N samples in a row
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(pin);
  }

  // average all the samples out
  averageDigitialCounts = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     averageDigitialCounts += samples[i];
  }
  averageDigitialCounts /= NUMSAMPLES;

  // We don't need it for the resistance equation but it will help for calibration
  // to report the microVolts
  int32_t microVolts = (int32_t)((averageDigitialCounts / 4095) * 3300000);
  gatt.setChar(sensorService->microVoltsCharId, microVolts);

  // Serial.print("Average analog reading ");
  // Serial.println(average);

  // convert the value to resistance
  // Solving the voltage divider equation: Vout = Vin * R2 / (R1 + R2)
  // For R2 yields: R2 = R1 / ((Vin / Vout) - 1)

  float resistanceRatio = (4095 / averageDigitialCounts) - 1;
  float resistance = coefficients->seriesResistance  / resistanceRatio;

  // Serial.print("Thermistor resistance ");
  // Serial.println(average);

  float steinhart;
  steinhart = resistance / coefficients->thermistorNominalResistance;     // (R/Ro)
  steinhart = log(steinhart);                                          // ln(R/Ro)
  steinhart /= coefficients->thermistorBeta;                           // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);                    // + (1/To)
  steinhart = 1.0 / steinhart;                                         // Invert
  steinhart -= 273.15;                         // convert to C

  Serial.print(F("Temperature "));
  Serial.print(label);
  Serial.print(F(" "));
  Serial.print(steinhart);
  Serial.println(F(" *C"));

  // convert temperature to 32bit integer with 2 decimals of precision
  // need to verify that negative numbers work
  int32_t temperature_100 = (int32_t)(steinhart * 100);
  Serial.print(F("Temperature*100 "));
  Serial.println(temperature_100);

  gatt.setChar(sensorService->measureCharId, (int32_t)(steinhart * 100));

}

void loop(void) {
  uint8_t i;
  float average;

  // every 2 seconds check for updates from the BLE module this is currently
  // only used for the name characteristic, but we could use it for the isConnected as well
  // and better yet we could use the DFU IRQ approach so no polling of the module is necessary
  ble.update(2000);

  // This is useful for testing the reading and writing of characteristics without all of the other
  // info printed in the console
  //  if (1) {
  //    delay(1000);
  //    return;
  //  }

  if(!ble.isConnected()){
    disconnectedCount += 1;
    Serial.print("No connection. Count: ");
    Serial.println(disconnectedCount);
    if(disconnectedCount > 60){
      // Serial.println("No connection before timeout, should sleep here");

      // We might be able to use the DFU interrupt mode so we are woken up by the BLE when there is
      // a connection event. This would save power, when we are not connected.

      // Seems the best option I can find for sleep is using rtc standby:
      // https://github.com/cavemoa/Feather-M0-Adalogger/blob/master/SimpleSleepUSB/SimpleSleepUSB.ino
    }
    delay(2000);
    // do not do any analog collection until we are connected
    return;
  }

  // if we are here then we are connected
  disconnectedCount = 0;

  // Update battery reading
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 4096; // convert to voltage in 12bit mode
  // the voltage should range between 3 and 4.2 (but might go higher)
  uint8_t percentage = (uint8_t)((measuredvbat - 3.0) / 1.25 * 100);

  Serial.print("Update battery level voltage: ");
  Serial.print(measuredvbat);
  Serial.print(" percent: ");
  Serial.println(percentage);
  battery.update(percentage);

  readTemperature(&sensorServiceA, F("A"), THERMISTORPINA, &userConfigV1.sensorA);
  readTemperature(&sensorServiceB, F("B"), THERMISTORPINB, &userConfigV1.sensorB);

  delay(500);
}
