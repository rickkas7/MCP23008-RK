#include "Particle.h"

#include "MCP23008-RK.h"

// GP7 is pin 17, next to the VDD pin on the top left
// GP0 is pin 10 on the top right

MCP23008 gpio(Wire, 0);

const unsigned long periods[] = { 500, 333, 100, 10 };

unsigned long PERIOD_MS = 10;
unsigned long pinTimes[sizeof(periods) / sizeof(periods[0])];
bool pinState[sizeof(periods) / sizeof(periods[0])];

void setup() {
	Serial.begin(9600);

	pinMode(D7, OUTPUT);

	gpio.begin();
	for(size_t ii = 0; ii < sizeof(periods) / sizeof(periods[0]); ii++) {
		gpio.pinMode(ii, OUTPUT);
		pinTimes[ii] = 0;
		pinState[ii] = false;
	}

	gpio.pinMode(7, INPUT_PULLUP);
}

void loop() {
	for(size_t ii = 0; ii < sizeof(periods) / sizeof(periods[0]); ii++) {
		if (millis() - pinTimes[ii] >= periods[ii]) {
			pinTimes[ii] = millis();

			pinState[ii] = !pinState[ii];
			gpio.digitalWrite(ii, pinState[ii]);
		}
	}

	digitalWrite(D7, gpio.digitalRead(7));
}
