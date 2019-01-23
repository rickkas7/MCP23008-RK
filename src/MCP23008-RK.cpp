#include "Particle.h"
#include "MCP23008-RK.h"


MCP23008::MCP23008(TwoWire &wire, int addr) :
	wire(wire), addr(addr) {
}

MCP23008::~MCP23008() {
}

void MCP23008::begin() {
	wire.begin();
}

void MCP23008::pinMode(uint16_t pin, PinMode mode) {
	if (!pinAvailable(pin)) {
		return;
	}

	// Set input or output mode
	// true for INPUT or INPUT_PULLUP or false for OUTPUT
	writeRegisterPin(REG_IODIR, pin, (mode != OUTPUT));

	// Set pull-ups (100K)
	if (mode == INPUT || mode == INPUT_PULLUP) {
		// true for INPUT_PULLUP, false for INPUT
		writeRegisterPin(REG_GPPU, pin, (mode == INPUT_PULLUP));
	}
}

PinMode MCP23008::getPinMode(uint16_t pin) {
	if (!pinAvailable(pin)) {
		return INPUT;
	}

	if (readRegisterPin(REG_IODIR, pin)) {
		// bit is 1 (INPUT)
		if (readRegisterPin(REG_GPPU, pin)) {
			// bit is 1
			return INPUT_PULLUP;
		}
		else {
			return INPUT;
		}
	}
	else {
		// bit is 0
		return OUTPUT;
	}
}

bool MCP23008::pinAvailable(uint16_t pin) {
	return pin < NUM_PINS;
}

void MCP23008::digitalWrite(uint16_t pin, uint8_t value) {
	if (!pinAvailable(pin)) {
		return;
	}
	writeRegisterPin(REG_OLAT, pin, (bool)value);
}

int32_t MCP23008::digitalRead(uint16_t pin) {
	if (!pinAvailable(pin)) {
		return HIGH;
	}

	if (readRegisterPin(REG_GPIO, pin)) {
		return HIGH;
	}
	else {
		return LOW;
	}
}

uint8_t MCP23008::readAllPins() {
	return readRegister(REG_GPIO);
}


bool MCP23008::readRegisterPin(uint8_t reg, uint16_t pin) {
	if (!pinAvailable(pin)) {
		return false;
	}

	return readRegister(reg) & (1 << pin);
}

bool MCP23008::writeRegisterPin(uint8_t reg, uint16_t pin, bool value) {
	if (!pinAvailable(pin)) {
		return false;
	}

	uint8_t regValue = readRegister(reg);

	if (value) {
		regValue |= (1 << pin);
	}
	else {
		regValue &= ~(1 << pin);
	}

	return writeRegister(reg, regValue);
}


uint8_t MCP23008::readRegister(uint8_t reg) {
	wire.beginTransmission(addr | DEVICE_ADDR);
	wire.write(reg);
	wire.endTransmission(false);

	wire.requestFrom(addr | DEVICE_ADDR, 1, true);
	uint8_t value = (uint8_t) wire.read();

	// Serial.printlnf("readRegister reg=%d value=%d", reg, value);

	return value;
}

bool MCP23008::writeRegister(uint8_t reg, uint8_t value) {
	wire.beginTransmission(addr | DEVICE_ADDR);
	wire.write(reg);
	wire.write(value);

	int stat = wire.endTransmission(true);

	// Serial.printlnf("writeRegister reg=%d value=%d stat=%d read=%d", reg, value, stat, readRegister(reg));
	return (stat == 0);
}

