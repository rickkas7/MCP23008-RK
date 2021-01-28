#include "Particle.h"

#include "MCP23008-RK.h"

SerialLogHandler logHandler;

MCP23008 gpio(Wire3, 0);

uint16_t SWITCH_PIN = 2; // GP2

void setup() {
    waitFor(Serial.isConnected, 15000);

    // Turn on power on Tracker CAN_5V
    pinMode(CAN_PWR, OUTPUT);
    digitalWrite(CAN_PWR, HIGH);
    delay(200);

	gpio.begin();

    // When using interrupt mode, you need to assocate a physical MCU pin as an interrupt pin
    // from the MCP23008 INT output
    gpio.enableInterrupts(A3, MCP23008InterruptOutputType::OPEN_DRAIN);

    gpio.pinMode(SWITCH_PIN, INPUT_PULLUP);
    
    gpio.attachInterrupt(SWITCH_PIN, CHANGE, [](bool bValue) {
        // This code runs in a worker thread with a 1024 byte stack, so avoid doing
        // anything that requires a long time or stack here.
        Log.info("bValue=%d", bValue);
    });

}

void loop() {
}
