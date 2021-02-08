const int THROTTLE_PIN = 10;

void setup()
{
  //hack to get 32khz pwm and smoother throttle signal. info: https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
  TCCR1B = TCCR1B & 0b11111001;
}

void loop()
{

    analogWrite(THROTTLE_PIN, 45);
    delay(5000);
    analogWrite(THROTTLE_PIN, 211);
    delay(5000);

}
