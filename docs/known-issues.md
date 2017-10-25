The non-volitale memory on the device is sometimes lost. So the icon is lost.

The BLE module sometimes gets corrupted and needs to be reset before it will work
again. This is done by grounding the RST pad below the BLE module.

When programming the device the first time, the version characteristic fails to be
set right.

When the device is starting up, the BLE module wakes up advertising. And the main
sketch is no running yet. So any requests for temperature would not be working. Also
the sketch resets the module after startup, so any early connections will be lost.

One possible solution for this is for the sketch to list the GATT services and then
check if they match what we expect. Then only if some of the services are changed do
we reset the module.  However there is still a problem where someone could connect
during the first few seconds it is turned on.

Another solution is to disable the advertising when the sketch first starts and then
re-enable it after it gets setup.

The adafruit library for talking to the module does not handle an error while reading
the non-volitale memory.
