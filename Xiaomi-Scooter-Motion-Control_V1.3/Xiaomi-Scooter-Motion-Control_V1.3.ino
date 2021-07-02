#include <Arduino.h>
#include <arduino-timer.h>
#include <SoftwareSerial.h> 

//SETTINGS

//===========================================================================
//=============================Select model  ================================
//===========================================================================
//
// 0 = Xiaomi Mi Scooter Essential
// 1 = Xiaomi Mi Scooter 1S (Not tested)
// 2 = Xiaomi Mi Scooter Pro 2

#define SCOOTERTYPE 0

//===========================================================================
//=============================Motion behavior  =============================
//===========================================================================

// Boost timer, how long the motor will be powered after a kick. time in miliseconds.
int boosttimer_tier1 = 3000;
int boosttimer_tier2 = 5000; 
int boosttimer_tier3 = 7000; 
int kickdelay = 1000; //time before you can do an new kick after boost timer is expired.

// Smooth readings of the speedometer. The higher the number, the more the readings 
// will be smoothed, but the slower the step will respond to the kicks.
const int speedReadings = 20;

//===========================================================================
//=============================   Pin Settings  =============================
//===========================================================================

// Arduino pin where throttle is connected to (only pin 9 & 10 is ok to use)
const int THROTTLE_PIN = 10;
int LED_PCB = 13;

//TX & RX pin
SoftwareSerial SoftSerial(2, 3); // RX, TX

//END OF SETTINGS

auto timer_m = timer_create_default();

int BrakeHandle;
int Speed;                      //current speed
int oldspeed;                   //speed during last loop
int trend = 0;                  //speed trend
int readings[speedReadings];    // the readings from the speedometer
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int AverageSpeed = 0;           // the average speed over last X readings
int OldAverageSpeed = 0;   			// the average speed over last X readings in the last loop



//motionmodes
uint8_t motionstate = 0;
#define motionready 0
#define motionbusy 1
#define motionbreaking 2


void logByteInHex(uint8_t val)
{
//  if(val < 16)
//    Serial.print('0');
//
//  Serial.print(val, 16);
//  Serial.print(' ');
}

uint8_t readBlocking()
{
  while(!SoftSerial.available())
    delay(1);

  return SoftSerial.read();
}

void setup()
{
  pinMode(LED_PCB, OUTPUT);
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < speedReadings; thisReading++) {
    readings[thisReading] = 0;
  }
    
  Serial.begin(115200);
  SoftSerial.begin(115200);

  Serial.println("Starting Logging data...");

  TCCR1B = TCCR1B & 0b11111001;  //Set PWM of PIN 9 & 10 to 32 khz

  ThrottleWrite(45);
}

uint8_t buff[256];
void loop()
{

  int w = 0;

  
  
  while(readBlocking() != 0x55);
  if(readBlocking() != 0xAA)
    return;

  uint8_t len = readBlocking();
  buff[0] = len;
  if(len > 254)
    return;

  uint8_t addr = readBlocking();
  buff[1] = addr;

  uint16_t sum = len + addr;
  for(int i = 0; i < len; i++)
  {
    uint8_t curr = readBlocking();
    buff[i + 2] = curr;
    sum += curr;
  }


  uint16_t checksum = (uint16_t)readBlocking() | ((uint16_t)readBlocking() << 8);
  if(checksum != (sum ^ 0xFFFF))
    return;

  for(int i = 0; i < len + 2; i++)
    logByteInHex(buff[i]);

//  Serial.print("check ");
//  Serial.print(checksum, 16);
//
//  Serial.println();
switch (buff[1]) {
  case 0x20:
    switch (buff[2]) {
      case 0x65:
        BrakeHandle = buff[6];
    }
   case 0x21:
    switch (buff[2]) {
      case 0x64:
        if (buff[8] != 0){
          Speed = buff[8];
        }
    } 
}


  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the speedometer:
  readings[readIndex] = Speed;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= speedReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  AverageSpeed = total / speedReadings;



Serial.print("Speed: ");
Serial.print(AverageSpeed);
Serial.print(" Brake: ");
Serial.print(BrakeHandle);
Serial.print(" State: ");
Serial.print(motionstate);
Serial.println(" ");

  motion_control();
  timer_m.tick();


}


bool release_throttle(void *) {
  Serial.println("Timer expired, stopping...");
  
  #if (SCOOTERTYPE==0)
  ThrottleWrite(80); //Keep throttle open for 10% to disable KERS. best for essential.
  #elif (SCOOTERTYPE==1) || (SCOOTERTYPE==2)
  ThrottleWrite(45); //Close throttle. best for pro 2 & 1S.
  #endif
  
  timer_m.in(kickdelay, motion_wait);
  return false; // false to stop
}

bool motion_wait(void *) {
  Serial.println("Ready for new kick!");
  motionstate = motionready;
  return false; // false to stop
}

void motion_control() {



      if ((Speed != 0) && (Speed < 5)) {
        // If speed is under 5 km/h, stop throttle
        ThrottleWrite(45); //  0% throttle
        
      }

  if (BrakeHandle > 47) {

    ThrottleWrite(45); //close throttle directly when break is touched. 0% throttle
    Serial.println("BRAKE detected!!!");
    digitalWrite(LED_PCB, HIGH);
    motionstate = motionready;
    timer_m.cancel();
  }
  else
  {
    digitalWrite(LED_PCB, LOW);
  }


if (Speed != 0){
          // Check if auto throttle is off and speed is increasing
  if (motionstate == motionready) {

      trend = AverageSpeed - OldAverageSpeed;

    if (trend > 0)
    {
      // speed is increasing
      // Check if speed is at least 5 km/h
      if (AverageSpeed > 5) {
        // Open throttle for 5 seconds
        AnalyseKick(); 
        Serial.println("Kick detected!");
        motionstate = motionbusy;

      }

    }
    else if (trend < 0)
    {
      // speed is decreasing

    }
    else
    {     
      // no change in speed
    }

  }

  oldspeed = Speed;
  OldAverageSpeed = AverageSpeed;
      }

}

void AnalyseKick(){

   if (AverageSpeed < 10) {
    ThrottleWrite(140); //  40% throttle
    timer_m.in(boosttimer_tier1, release_throttle); //Set timer to release throttle
   }
    else if ((AverageSpeed >= 10) & (AverageSpeed < 14)){
      ThrottleWrite(190); //  80% throttle
      timer_m.in(boosttimer_tier2, release_throttle); //Set timer to release throttle
    }else{
    
    ThrottleWrite(233); //  100% throttle
    timer_m.in(boosttimer_tier3, release_throttle); //Set timer to release throttle
   }
  
  
}


int ThrottleWrite(int value)
{
  if (value != 0){
      analogWrite(THROTTLE_PIN, value);

  }

}
