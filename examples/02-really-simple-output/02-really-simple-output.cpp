#include "Particle.h"

#include "MCP23008-RK.h"

// GP7 is pin 17, next to the VDD pin on the top left
// GP0 is pin 10 on the top right

MCP23008 gpio(Wire, 0);


void setup() {
	Serial.begin(9600);

	delay(5000);

	gpio.begin();
	gpio.pinMode(0, OUTPUT);
	gpio.digitalWrite(0, HIGH);
}

void loop() {
}
