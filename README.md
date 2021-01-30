# Xiaomi-Scooter-Motion-Control
Modification to legalise the Xiaomi Mi Scooters in The Netherlands (Project is not ready to use yet! Please wait before buying parts as they might change.)

The code of this project is based of:
https://github.com/augisbud/m365_dashboard
So actually m365 dashboard with some extra code. The dashboard display is only used for debugging at the moment.

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

If you want to use an M365 you might need lower the motor output. Not sure if the motor is 250 watts.


# How it works

An Arduino Nano will be used to read out the serial-bus of the Xiaomi Mi Scooter.
The speedometer will be monitored if there are any the push offs with your feed. When there is a push, the throttle will be opened to 100% for 5 seconds and then goes to 1% (0% is regen breaking).
When the brakehandle is being touched the throttle will be released immediately. Also the Mi scooter itself disables the throttle also in case of braking.


# Hardware

- Arduino Nano
- JST-ZH cord plug (not sure) to replace the throttle. (or cut it from the trottle, a new one is 4 euro)
- If you don't want to solder on your xiaomi: A male and female 4-pole e-bike plug like: https://nl.aliexpress.com/item/4001091169417.html
- Diode: 1N148
- Resistor: 120R

# Wiring

![alt text](https://github.com/PsychoMnts/Xiaomi-Scooter-Motion-Control/blob/main/Wiring%20Scheme.png?raw=true)
Scheme is not final

# To-do
- Purchase test scooter
- Test the code
- Clean the code

