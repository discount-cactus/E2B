//Board-to-Board example - Slave node
/*NOTES:
-For ATtiny85, the Serial.print and Serial.println lines will have to be removed
-For ATtiny85, modify E2B_config.h to clear up more program memory space
*/
#include <E2B.h>

#define E2B_pin 2

//unsigned char rom[8];
unsigned char rom[8] = {FAMILYCODE, 0xE2, 0xCC, 0x2D, 0x01, 0x25, 0xB8, 0x30};
unsigned char scratchpad[9] = {0x5E, 0x00, 0x00, 0x04, 0x00, 0x31, 0x6A, 0xD7, 0x00};

E2B e2b(E2B_pin);  // on pin 2 (a 4.7K resistor is necessary)

void setup() {
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("E2B Slave Node Test.");
  //randomSeed(analogRead(0));          //Uncomment when generating new rom address
  //e2b.generateROM(rom);               //Uncomment when generating new rom address
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);

  //e2b.setDeviceType(POINTTOPOINT);    //Can use this when only one device is connected
}

void respond(){
  e2b.MasterResetPulseDetection();
}

void loop(void) {
  e2b.waitForRequest(false);
  Serial.println(e2b.getScratchpad(4),HEX);
}
