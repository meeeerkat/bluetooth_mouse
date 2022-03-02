
## A bluetooth mouse peripheric
Made with an ESP32 devkit and some cables from the esp32 bluetooth mouse example.
It's not practical to use the peripheric, this is project's purpose was only to learn with the few electronic components I had.

### How to use it
There are 6 touch "keys" that either move the cursor (left, right, up or down) or click (left or right).
These keys are touch io inputs so there is no need to use buttons.
One can also change the cursor's speed with a potentiometer (on GPIO34).

### Configuration
All keys' pin number can be configured and the cursor's speed min and max value too.
The refresh delta is the time in milliseconds between each command sent to the computer.

### Current state
Moving the cursor and clicking (right and left clicks) work.
Using a potentiometer on GPIO34 can set the cursor's speed.


### TODO
- Add the GPIO for the cursor's speed potentiometer in the menuconfig
- Add mouse wheel (with a potentiometer)
- Separate the different parts of bt\_hid\_task

### Remarks
I'm using the touch functionnality because I simply don't have enough buttons (and it's more more convenient to setup/use ?).
