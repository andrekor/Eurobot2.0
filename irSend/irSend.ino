#include "IRremote.h";

int h = 38;
//the IR remote library adds a start sequence 010101 in the beginning of the signal
unsigned int buf[6] = {600, 600, 1200, 600, 1200, 600};

IRsend irsend;

void setup() {
}

void loop() {
	//irsend.sendSony(0x0000, 16);
  	irsend.sendRaw(buf, 5, h);	
    delay(5);
}