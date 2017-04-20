typedef struct ATTR_PACKED TempCoefficients {  
  // resistance of the series resistor default to 10 kOhm
  uint16_t seriesResistance;
  // thermistor resistance at 25 degrees default to 10 kOhm
  uint16_t thermistorNominalResistance;
  // thermistor beta coefficient (usually 3000-4000) default to 3950
  uint16_t thermistorBeta;
} TempCoefficients;

// This is currently 17 bytes
#define ID_STRING_LEN 5
typedef struct ATTR_PACKED UserConfigV1 {
  char idString[ID_STRING_LEN];
  TempCoefficients sensorA;
  TempCoefficients sensorB;
} UserConfigV1;


// place to store info about the gatt service and characteristics
typedef struct SensorServiceInfo {
  int32_t serviceId;
  int32_t measureCharId;  
  int32_t voltsCharId;
  int32_t seriesResistanceCharId;
  int32_t thermistorNominalResistanceCharId;
  int32_t thermistorBetaCharId;
} SensorServiceInfo;

// Need prototypes here because arduino is automatically adding these to the top of the file which is before the structs
// are defined
void readTemperature(SensorServiceInfo *sensorService, const __FlashStringHelper*label, int32_t pin, TempCoefficients *coefficients);
void addSensorService(uint8_t serviceUUID[], SensorServiceInfo *info);

