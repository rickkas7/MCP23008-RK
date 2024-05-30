#include "Particle.h"

#include "MCP23008-RK.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// GP7 is pin 17, next to the VDD pin on the top left
// GP0 is pin 10 on the top right

MCP23008 gpio(Wire, 0);

const unsigned long loopDelayMs = 1;
Thread *workerThread;
os_thread_return_t threadFunction(void);
bool pinState = false;

void setup() {
	Serial.begin(9600);

	gpio.begin();
	gpio.pinMode(0, OUTPUT);

    workerThread = new Thread("MyClass", threadFunction, OS_THREAD_PRIORITY_DEFAULT, 3072);
}

void loop() {
}

os_thread_return_t threadFunction(void) {
    while(true) {
        pinState = !pinState;
        gpio.digitalWrite(0, pinState);

        delay(loopDelayMs);
    }
}
