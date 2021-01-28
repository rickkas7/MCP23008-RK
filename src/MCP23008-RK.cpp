#include "Particle.h"
#include "MCP23008-RK.h"

Thread *MCP23008::thread;
std::vector<MCP23008 *> *MCP23008::instances;


MCP23008::MCP23008(TwoWire &wire, int addr) :
	wire(wire), addr(addr) {
	
	if (!instances) {
		instances = new std::vector<MCP23008 *>();
	}
	instances->push_back(this);
}

MCP23008::~MCP23008() {
	while(!interruptHandlers.empty()) {
		MCP23008InterruptHandler *h = interruptHandlers.back();
		interruptHandlers.pop_back();

		delete h;
	}

	// Remove from instances list
	for(auto it = instances->begin(); it != instances->end(); it++) {
		if (this == *it) {
			instances->erase(it);
			break;
		}
	}
}

void MCP23008::begin() {
	wire.begin();

	// Disable interrupts until explicitly set
	writeRegister(REG_GPINTEN, 0);
	writeRegister(REG_INTCON, 0);
}

void MCP23008::pinMode(uint16_t pin, PinMode mode) {
	if (!pinAvailable(pin)) {
		return;
	}

	wire.lock();
	// Set input or output mode
	// true for INPUT or INPUT_PULLUP or false for OUTPUT
	writeRegisterPin(REG_IODIR, pin, (mode != OUTPUT));

	// Set pull-ups (100K)
	if (mode == INPUT || mode == INPUT_PULLUP) {
		// true for INPUT_PULLUP, false for INPUT
		writeRegisterPin(REG_GPPU, pin, (mode == INPUT_PULLUP));
	}
	wire.unlock();
}

PinMode MCP23008::getPinMode(uint16_t pin) {
	if (!pinAvailable(pin)) {
		return INPUT;
	}

	wire.lock();
	bool ioDir = readRegisterPin(REG_IODIR, pin);
	bool gppu = readRegisterPin(REG_GPPU, pin);
	wire.unlock();

	if (ioDir) {
		// bit is 1 (INPUT)
		if (gppu) {
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
	wire.lock();
	writeRegisterPin(REG_OLAT, pin, (bool)value);
	wire.unlock();
}

int32_t MCP23008::digitalRead(uint16_t pin) {
	if (!pinAvailable(pin)) {
		return HIGH;
	}

	wire.lock();
	bool value = readRegisterPin(REG_GPIO, pin);
	wire.unlock();

	if (value) {
		return HIGH;
	}
	else {
		return LOW;
	}
}

uint8_t MCP23008::readAllPins() {
	wire.lock();
	uint8_t result = readRegister(REG_GPIO);
	wire.unlock();

	return result;
}

void MCP23008::enableInterrupts(pin_t pin, MCP23008InterruptOutputType outputType) {
	if (!thread) {
	    new Thread("mcp23008", threadFunctionStatic, this, OS_THREAD_PRIORITY_DEFAULT, stackSize);
	}
	mcuInterruptPin = pin;

	wire.lock();

	uint8_t reg = readRegister(REG_IOCON);

	if (outputType == MCP23008InterruptOutputType::OPEN_DRAIN || 
		outputType == MCP23008InterruptOutputType::OPEN_DRAIN_NO_PULL) {
		if (outputType == MCP23008InterruptOutputType::OPEN_DRAIN) {
			::pinMode(pin, INPUT_PULLUP);
		}
		else {
			::pinMode(pin, INPUT);
		}
		// Set ODR bit
		reg |= 0b100;

		// INTPOL is ignored when ODR is set
	}
	else {
		// Output is push-pull
		::pinMode(pin, INPUT);

		// Clear ODR bit
		reg &= ~0b100;

		if (outputType == MCP23008InterruptOutputType::ACTIVE_HIGH) {
			// Set INTPOL
			reg |= 0b010;
		}
	}

	writeRegister(REG_IOCON, reg);

	wire.unlock();

}

void MCP23008::attachInterrupt(uint16_t pin, InterruptMode mode, std::function<void(bool)> handler, void *context) {
	detachInterrupt(pin);

	MCP23008InterruptHandler *h = new MCP23008InterruptHandler();
	h->pin = pin;
	h->mode = mode;
	h->handler = handler;
	h->context = context;
	h->lastState = digitalRead(pin);

	interruptHandlers.push_back(h);

	// Enable this pin for interrupts
	wire.lock();
	uint8_t reg = readRegister(REG_GPINTEN);
	reg |= 1 << pin;
	writeRegister(REG_GPINTEN, reg);
	wire.unlock();

}

void MCP23008::detachInterrupt(uint16_t pin) {
	// Disable this pin for interrupts
	wire.lock();
	uint8_t reg = readRegister(REG_GPINTEN);
	reg &= ~(1 << pin);
	writeRegister(REG_GPINTEN, reg);
	wire.unlock();

	// Remove from the vector of handlers
	for(auto it = interruptHandlers.begin(); it != interruptHandlers.end(); it++) {
		MCP23008InterruptHandler *h = *it;
		if (h->pin == pin) {
			interruptHandlers.erase(it);
			delete h;
			break;
		}
	}
}

void MCP23008::handleInterrupts() {
	// This runs from a worker thread!

	if (mcuInterruptPin == PIN_INVALID || pinReadFast(mcuInterruptPin) == HIGH) {
		// No interrupt
		return;
	}

	wire.lock();

	// Read the INTF register. A bit will be 1 if the port caused an interrupt
	uint8_t intFlag = readRegister(REG_INTF);

	// Reads the values 
	uint8_t intValue = readRegister(REG_INTCAP);

	wire.unlock();

	// Log.info("intFlag=%02x intValue=%02x intf after=%02x", intFlag, intValue, readRegister(REG_INTF));

	for(auto it = interruptHandlers.begin(); it != interruptHandlers.end(); it++) {
		MCP23008InterruptHandler *h = *it;

		if (((1 << h->pin) & intFlag) != 0) {
			// This pin caused an interrupt
			bool bValue = (intValue & (1 << h->pin)) != 0;

			switch(h->mode) {
			case RISING:
				if (bValue) {
					h->handler(bValue);
				}
				break;

			case FALLING:
				if (!bValue) {
					h->handler(bValue);
				}
				break;

			case CHANGE:
				h->handler(bValue);
				break;
			}
		}
	}
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

// static 
os_thread_return_t MCP23008::threadFunctionStatic(void* param) {
	while(true) {
		for(auto it = instances->begin(); it != instances->end(); it++) {
			MCP23008 *m = *it;
			m->handleInterrupts();
		}

		os_thread_yield();
	}
}

