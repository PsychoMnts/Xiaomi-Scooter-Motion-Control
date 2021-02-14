#include <Arduino.h>

void logByteInHex(uint8_t val)
{
	if(val < 16)
		Serial.print('0');

	Serial.print(val, 16);
	Serial.print(' ');
}

uint8_t readBlocking()
{
	while(!Serial2.available())
		delay(1);

	return Serial2.read();
}

void setup()
{
	Serial.begin(115200);
	Serial2.begin(115200);

	Serial.println("Starting Logging data...");
}

uint8_t buff[256];
void loop()
{

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

	Serial.print("check ");
	Serial.print(checksum, 16);

	Serial.println();
}
