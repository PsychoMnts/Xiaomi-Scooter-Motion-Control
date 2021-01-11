# Xiaomi-Scooter-Motion-Control
Modification to legalise the Xiaomi Mi Scooters in The Netherlands (Project is not ready to use yet)

The idea is to make an small hardware modification on the Xiaomi scooters so the comply with the Dutch law. 

To use an e-step in The Netherlands, you must comply with the following rules:
- There must NO throttle button.
- The motor must be limited to 250 watts.
- The motor can only give a "boost" when you push off with your feet.
- Max speed is 25 km/h.

Example:
Micro has e-steps which are modified to comply with dutch rules with an deactivated throttle, like the MICRO M1 COLIBRI.
https://www.micro-step.nl/nl/emicro-m1-colibri-nl.html
Example video:
https://www.youtube.com/watch?v=ig-_bv6khLc

The best scooter to do this modification is the Xiaomi Mi Electric Scooter Essential:
- The motor is 250 watts
- Max speed is 20 km/h, which is already fast to give push offs with your feet.

If you want to use an M365 you must modify the firmware to lower the motor output.


## Option 1

Use a accelerometer to measure the push offs. When there is a push, the throttle will be opened to 100% for a few seconds and slowly fades to 0%.
The benefit is that you don't have to solder on the wiring of the e-step.

## Option 2

Read out the serial wire. The M365 dashboard (https://github.com/augisbud/m365_dashboard) already figured out how to read the current speed.
This method is proberly more reliable. Measure if there is increasing speed when there is no throttle and then open the throttle with code and slowly fade it to 0%.

# Technical
## Throttle
The throttle cable can be connected to an MCP4151 which is an digital potential meter. The software should be programmed that it will only open the throttle when the e-step goes faster than 5 km/h. Opening the throttle before that it won't power the motor.
The break always wins when being pressed. When the throttle is 100% open, it should be lowered to 0% before you can power the motor again.

# To-do
- Measure the resistance on the potmeter of the throttle.
- Decide for the accelerometer option or the direct readout of the serial wire.
- Make a wiring scheme
- Write software (I am not a developer, so please help me out!)
