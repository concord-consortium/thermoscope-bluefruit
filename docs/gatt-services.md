# GATT Services

In addition to some default services provided by the Bluefuit board it provides
three additional services.  Two temperature services and one generic device service.

## Temperature Services

- f000aa0004514000b000000000000000 - Temperature A
- f000bb0004514000b000000000000000 - Temperature B

Each Temperature Service has 5 characteristics. 2 measurements and 3 calibration.

### Measurements

- 0x0001 - temperature Measurement - integer with the temp in C times 100 so 30.01 C is
  represented as 3001
- 0x0002 - int microvolts measurement as an integer

### Calibrations

- 0x0003 - int value of the series resistor in Ohms default is: 10000
- 0x0004 - int thermistor nominal resistance in Ohms default is: 10000
- 0x0005 - int thermistorBeta default is: 3950

## Generic Device Service

0x1234 - Info about device

### Identifier Icon

ID: 0x2345

It is used to store a uft8 character to identifier the device
this character is added onto the end of the device name "Thermoscope "

Updating the uft8 character with some BLE exploring apps is tricky. They don't handle
multi byte characters well. I used LightBlue an app on OS X, to write the characters.
First I converted the character to its hex bytes using this site: http://www.ltg.ed.ac.uk/~richard/utf-8.cgi
An example character is this one:
  http://www.ltg.ed.ac.uk/~richard/utf-8.cgi?input=%F0%9F%9A%80&mode=char
Then in LightBlue I found the 0x1234 service followed by 0x2345 characteristc and then
entered the hex value for the character in this case: F09F9A80. Hitting enter in LightBlue
sends the value to the device.  I'm not sure if LightBlue sends a null character in this
case, but I think it does not. I think it simply sends the 4 bytes.

### Version Info

ID: 0x6789

A string version identifier for the sketch.

## Battery Info

This is the standard GATT battery service with standard GATT battery characteristic.
The value is a percentage. It is calculated from the battery voltage:
(battery_voltage - 3.0V)/4.8V that formula works well for the 3 AAA batteries which
range from 1V to 1.6V.
