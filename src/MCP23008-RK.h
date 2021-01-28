#ifndef __MCP23008_RK_H
#define __MCP23008_RK_H

#include "Particle.h"

#include <vector>

enum class MCP23008InterruptOutputType {
	ACTIVE_LOW,
	ACTIVE_HIGH,
	OPEN_DRAIN,
	OPEN_DRAIN_NO_PULL
};

struct MCP23008InterruptHandler {
	uint16_t pin;
	InterruptMode mode;
	std::function<void(bool)> handler;
	void *context;
	bool lastState;
};

class MCP23008 {
public:
	MCP23008(TwoWire &wire, int addr = 0);
	virtual ~MCP23008();

	void begin();

	// Arduino-style API
	void pinMode(uint16_t pin, PinMode mode);
	PinMode getPinMode(uint16_t pin);
	bool pinAvailable(uint16_t pin);
	void digitalWrite(uint16_t pin, uint8_t value);
	int32_t digitalRead(uint16_t pin);

	uint8_t readAllPins();

	void enableInterrupts(pin_t mcuInterruptPin, MCP23008InterruptOutputType outputType);
	void attachInterrupt(uint16_t pin, InterruptMode mode, std::function<void(bool)> handler, void *context = 0);
	void detachInterrupt(uint16_t pin);

	bool readRegisterPin(uint8_t reg, uint16_t pin);
	bool writeRegisterPin(uint8_t reg, uint16_t pin, bool value);

	uint8_t readRegister(uint8_t reg);
	bool writeRegister(uint8_t reg, uint8_t value);


	// These are just for reference so you can see which way the pin numbers are laid out
	// (1 << pin) is also a good way to map pin numbers to their bit
	static const uint8_t PIN_0 = 0b00000001;
	static const uint8_t PIN_1 = 0b00000010;
	static const uint8_t PIN_2 = 0b00000100;
	static const uint8_t PIN_3 = 0b00001000;
	static const uint8_t PIN_4 = 0b00010000;
	static const uint8_t PIN_5 = 0b00100000;
	static const uint8_t PIN_6 = 0b01000000;
	static const uint8_t PIN_7 = 0b10000000;

	static const uint16_t NUM_PINS = 8;

	static const uint8_t REG_IODIR = 0x0;
	static const uint8_t REG_IPOL = 0x1;
	static const uint8_t REG_GPINTEN = 0x2;
	static const uint8_t REG_DEFVAL = 0x3;
	static const uint8_t REG_INTCON = 0x4;
	static const uint8_t REG_IOCON = 0x5;
	static const uint8_t REG_GPPU = 0x6;
	static const uint8_t REG_INTF = 0x7;
	static const uint8_t REG_INTCAP = 0x8;
	static const uint8_t REG_GPIO = 0x9;
	static const uint8_t REG_OLAT = 0xa;

	static const uint8_t DEVICE_ADDR = 0b0100000;

protected:
	void handleInterrupts();

	static os_thread_return_t threadFunctionStatic(void* param);

	pin_t mcuInterruptPin = PIN_INVALID;
	std::vector<MCP23008InterruptHandler*> interruptHandlers;
	TwoWire &wire;
	int addr; // This is just 0-7, the (0b0100000 of the 7-bit address is ORed in later)
	size_t stackSize = 1024;
	static Thread *thread;
	static std::vector<MCP23008 *> *instances;
};

#endif /* __MCP23008_RK_H */
