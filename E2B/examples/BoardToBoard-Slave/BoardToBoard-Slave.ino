//Board-to-Board example - Slave node
#include <E2B.h>

#define E2B_pin 2

unsigned char rom[8] = {FAMILYCODE, 0xE2, 0xCC, 0x2D, 0x01, 0x25, 0xB8, 0x30};
//unsigned char rom[8] = {FAMILYCODE, 0x88, 0x0F, 0xE1, 0x00, 0xA5, 0xBC, 0x9D};
unsigned char scratchpad[9] = {0x5E, 0x00, 0x00, 0x04, 0x00, 0x31, 0x6A, 0xD7, 0x00};
//unsigned char scratchpad[9] = {0x3C, 0x00, 0xAD, 0xAD, 0x1F, 0x00, 0x55, 0x06, 0xB0};

E2B ds(E2B_pin);  // on pin 10 (a 4.7K resistor is necessary)

void setup() {
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Test.");
  ds.init(rom);
  ds.setScratchpad(scratchpad);
}

void respond(){
  ds.MasterResetPulseDetection();
}

void loop(void) {
  ds.waitForRequest(false);
  Serial.println(ds.scratchpad[4],HEX);
}
