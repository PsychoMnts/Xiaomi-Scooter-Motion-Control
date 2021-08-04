# Xiaomi-Scooter-Motion-Control
Modification to legalise the Xiaomi Mi Scooters in The Netherlands.

The idea is to make an small hardware modification on the Xiaomi scooters so they comply with the Dutch law. 

To use an scooter (or how we call it here: e-step) in The Netherlands, you must comply with the following rules:
- There must NO throttle button.
- The motor must be limited to 250 watts.
- The motor can only give a "boost" when you kick with your feet.
- The motor should stop working when stop kicking. (fade-out)
- Max speed is 25 km/h.

Example:
Micro has e-steps which are modified to comply with dutch rules with an deactivated throttle, like the MICRO M1 COLIBRI.
https://www.micro-step.nl/nl/emicro-m1-colibri-nl.html


The best scooter to do this modification is the Xiaomi Mi Electric Scooter Essential:
- The motor is 250 watts
- Max speed is 20 km/h, which is already fast to give push offs with your feet.

# Librarys

- https://github.com/contrem/arduino-timer

# How it works

An Arduino Nano will be used to read out the serial-bus of the Xiaomi Mi Scooter.
The speedometer will be monitored if there are any kicks with your feed. When there is a kick, the throttle will be opened to 100% for 8 seconds and then goes to 10% (0% is regen breaking).
When the brakehandle is being touched the throttle will be released immediately. Also the Mi scooter itself disables the throttle also in case of braking.


# Hardware

- Arduino Nano
- 1k resistor
- 0.47uF Capacitor

If you don't want to solder on your scooter, you need also:

- JST-ZH male-plug. (or cut it from the trottle, a new one is 2 - 4 euro) https://nl.aliexpress.com/item/1005001992213252.html
- A male and female 4-pole e-bike plug like: https://nl.aliexpress.com/item/4001091169417.html (Blue plug)

Or just buy a complete set!
https://www.legaalsteppen.nl/p/motion-control-module-azdelivery-arduino-nano

# Wiring

![alt text](https://github.com/PsychoMnts/Xiaomi-Scooter-Motion-Control/blob/main/Wiring%20Scheme_v3.png?raw=true)

# Supported models
- XIAOMI Mi Electric Scooter Essential
- XIAOMI Mi Electric Scooter 1S EU

Limit motor modification required:
- XIAOMI Mi Electric Scooter M365 Pro
- XIAOMI Mi Electric Scooter Pro 2


Guide: https://github.com/PsychoMnts/Xiaomi-Scooter-Motion-Control/blob/main/How%20to%20limit%20the%20Xiaomi%20Pro2%20scooter.docx?raw=true

XIAOMI Mi Electric Scooter M365 without Speed-O-Meter is NOT compatible!

To help supporting more scooters, please use the sniffing tool and share the serial bus data. Join our telegram group if you want to help in this project. https://t.me/joinchat/IuIjHecjckhK1h-a


# IenW over steps met stepondersteuning

"We stellen ons echter op het standpunt dat een tweewielig voertuig, dat met eigen spierkracht wordt voortbewogen en dat duidelijk op het fietspad thuishoort, in de categorie fiets hoort te vallen. (...)  Door de aard van de ondersteuning vallen deze steppen dus ook in de categorie ‘fiets met trapondersteuning’ en hoeven ze niet apart als bijzondere bromfiets te worden toegelaten. U mag hiermee tot maximaal 25 km/u op de openbare weg rijden."

DE MINISTER VAN INFRASTRUCTUUR EN WATERSTAAT,

Namens deze,

Hoofd afdeling Verkeersveiligheid en Wegvervoer

drs. M.N.E.J.G. Philippens


