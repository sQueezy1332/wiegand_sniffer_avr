/*
 Name:		WIEGAND_SNIFFER.ino
 Created:	29-Nov-24 04:04:59
 Author:	sQueezy
*/
#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(x)
#define DEBUGLN(x) Serial.println(x)
#define DEBUGF(x, ...) Serial.printf(x , ##__VA_ARGS__)
#else
#define DEBUG(x)
#endif
#define uS micros()
#define CARD_LEN 24     // minimum card length in bits
#define PULSE_WIDTH 100 // length of asserted pulse in microSeconds
#define PULSE_DELTA 2000   // delay between pulses in microSeconds
#define PIN_D0 2
#define PIN_D1 3
#include <EEPROM.h>
#define LIMIT 100
#define keylen 8 
#define MASTERKEY (0x0FFFFF)

#define readByte(addr) EEPROM.read(addr)
#define writeByte(addr, value) EEPROM.update(addr, value)
#define readKey(addr, ptr) EEPROM.get((addr) * keylen, ptr)
#define writeKey(addr, ptr) EEPROM.put((addr) * keylen, ptr)
#define readBlock(addr, ptr) EEPROM.get((addr), ptr)
#define writeBlock(addr, ptr) EEPROM.put((addr), ptr)
#define writedKeysByte (E2END)
#define periodByte (E2END - 2)

static volatile uint64_t	bitstream = 0;
static volatile uint32_t 	lastWiegand = 0;
static volatile uint32_t	reader_delta = 0;
static volatile byte		bitCount = 0;
static const byte LIMITKEYS = (E2END / keylen); //127 
static byte writedKeys = 0;

typedef volatile struct {
	int bitCount : 6;
	int Tpulse : 6;
	int Tfull : 10;
	long long code : 42;
}wieg_t;
//wieg_t wg;

ISR(INT0_vect) {
	auto time = uS;
	if (lastWiegand) reader_delta += time - lastWiegand;
	lastWiegand = time;
	bitCount++;				// Increament bit count for Interrupt connected to D0
	bitstream <<= 1;				// D0 represent binary 0, so just left shift card data 
}
ISR(INT1_vect) { __vector_1(); bitstream |= 1; }

void reset_vars() {
	bitCount = reader_delta = lastWiegand = bitstream = 0;
}

void transmit_assert(bool bit, uint16_t pulse_delta) {
	bit ? digitalWrite(PIN_D1, LOW) : digitalWrite(PIN_D0, LOW);
	delayMicroseconds(PULSE_WIDTH);
	bit ? digitalWrite(PIN_D1, HIGH) : digitalWrite(PIN_D0, HIGH);
	delayMicroseconds(pulse_delta);
}

void wiegand_emulate(uint64_t sendValue, uint16_t pulse_period) {
	byte bitcount = ((byte*)&sendValue)[7]; if (bitcount > 56) return;
	noInterrupts(); EIMSK = 0; pulse_period -= PULSE_WIDTH;
	for (uint64_t bitmask = 1 << (bitcount - 1); bitmask; bitmask >>= 1) {
		transmit_assert(sendValue & bitmask, pulse_period);
	}
	EIMSK = bit(INT0) | bit(INT1); interrupts();
};

void writeKeys(uint64_t key) {
	const byte oldWritedKeys = writedKeys; byte block, match;
	if (oldWritedKeys == 0) goto write;
	if (oldWritedKeys < LIMITKEYS) {
		for (block = 0; block < oldWritedKeys; block++) {  // checking for key not exist in eeprom
			for (match = 0;;) {
				if (((byte*)&key)[match] != readByte(block * keylen + match)) break;
				if (++match == keylen) return;      //already exist in eeprom
			}
		}
write:	Serial.println(F("\t\tWRITE"));
		writeKey(writedKeys++, key);
		writeByte(writedKeysByte, writedKeys);
		return;
	}
}

bool masterKeyCheck() {
	uint32_t key = bitstream & 0x1FFFFFF;
	if (bitCount & 3) key >>= 1; //%4
	if (key >= MASTERKEY) return true;
	return false;
}

void setup() {
#ifdef DEBUG_ENABLE
	Serial.begin(115200);
#endif
	delay(2000);
	Serial.println(F("\nSTART"));
	const byte _count = readByte(writedKeysByte);
	if (_count != 0xFF) { 
		writedKeys = _count; 
		uint64_t card; uint16_t Tdelta;
		readBlock(periodByte, Tdelta);
		Serial.print(F("\tWrited keys = ")); Serial.println(_count);
		Serial.print(F("\tPulse period = ")); Serial.println(Tdelta);
		for (byte i = 0, bitcount, j, BYTE; i < _count; i++) {
			readBlock(i * keylen, card);
			bitcount = ((byte*)&card)[7];
			j = bitcount >> 3; // /8
			if(bitcount & 7) ++j; //%8
			do {
				BYTE = ((byte*)&card)[j];
				if ((BYTE & 0xF0) == 0) Serial.print('0');
				Serial.print(BYTE, HEX); /*Serial.print(' ');*/
			} while (j--);
			Serial.print("\t:"); Serial.println(bitcount);
		}
	}
	pinMode(PIN_D0, INPUT_PULLUP);					// Set D0 pin as input
	pinMode(PIN_D1, INPUT_PULLUP);					// Set D1 pin as input
	EIMSK = (bit(INT0) | bit(INT1)); EICRA = bit(ISC01) | bit(ISC11); //FALLING
}

void loop() {
	if (bitCount >= CARD_LEN && bitstream && (uS - lastWiegand > 5000)) {
		EIMSK = 0; EIFR = 0b11;
		if (writedKeys == 0 && readByte(periodByte) == 0xFF && readByte(periodByte + 1) == 0xFF) {
			uint16_t Tperiod = reader_delta / (bitCount - 1); writeBlock(periodByte, Tperiod);
		}
		if (!masterKeyCheck()) {
			((byte*)&bitstream)[7] = bitCount;
			writeKeys(bitstream);
		} else {
			uint64_t card; uint16_t Tperiod;
			for (byte i = 0; i < writedKeys; i++) {
				readBlock(i * keylen, card);
				readBlock(periodByte, Tperiod);
				if (Tperiod > 3000) Tperiod = 3000;
				delay(1000);
				wiegand_emulate(card, Tperiod);
			}
		}
		reset_vars();
		EIMSK = bit(INT0) | bit(INT1);
	}
	delay(100);
}

uint64_t GetCardId() {
	if (bitCount == 26)	return bitstream & 0x1FFFFFE;		// EM tag
	if (bitCount == 34)	return bitstream & 0x1FFFFFFFE;		// Mifare 
	if (bitCount == 37) return bitstream & 0xFFFFFFFFE;
	if (bitCount == 42) return bitstream & 0x1FFFFFFFFFE;
	return bitstream;										// EM tag or Mifare without parity bits
};


