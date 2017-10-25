# LED Flash Patterns

There are 2 sets of LEDS on the board
- Main board LEDS next to the micro usb connector
- BLE Module LEDS next to the BLE module

## BLE Module LEDS

These are on either side of metal rectangular piece opposite from the micro usb connector.

The red LED next to the BLE module flashes 3 times times and pauses then repeats. It is
sending advertising packets or it is connected to computer.

The blue LED next to the BLE module lights up when a computer is connected.

## Main Board LEDS

There is a yellow and red LED. The yellow LED indicates the battery is being charged.
It should only light up if a usb cable is plugged in, so generally it can be ignored.

The normal power on pattern for the red LED is:

- solid on approx. 2 seconds: the chip is starting up, our sketch hasn't started yet
- 0.5 s off: the sketch just started
- solid on approx. 2 seconds: the sketch is running its setup tasks
- off: the sketch is fully setup and the bluetooth module is configured

If there is an error, then the code will stop running and the LED will flicker 10 times
a second.

### Important Note

The last step of the sketch setup is to reset the bluetooth module. This means that
any connections to the module will be dropped. **If you connect to the Thermoscope
while the main board red LED is lit up, your connection will be dropped.**

## Troubleshooting

If you see the fast flickering red main board LED, look to see if the BLE red LED
is flashing its stutter pattern.

If the BLE red LED is flashing the stutter pattern, and you see a flast flickering
main board LED, power cycling the device should help. If this does not help, then
the board will need to be reprogrammed.

If the BLE red LED is not flashing, and you see a flash flickering main board LED,
then it usually requires hardware intervention. First try to power cycle the device
and see if that helps. If not, then solution is:

1. Get a small alligator clip.
2. Turn off the board.
3. Clip one end to ground. The shell around the micro usb connector is a good place.
4. Look at the back of the board underneath the BLE module. Touch the other end of
the alligator clip to the pad labeled RST.
5. While the RST pad is grounded, power on the board.
6. Disconnect the alligator clip.
7. The BLE module LED should now be flashing the stutter pattern.
8. The icon on the device will need to be reset after this process
