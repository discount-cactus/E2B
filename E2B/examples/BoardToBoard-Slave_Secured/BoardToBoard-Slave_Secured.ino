//Board-to-Board example - Secured Slave node
#include <E2B.h>
/* NOTES:
Secured slave nodes cannot be communicated with until unlocked by an unlock(KEY) command
where the device is unlocked to receive one command before it is automatically locked again

e2b.unlock(uint8_t KEY): KEY is the key you send with an unlock command to unlock a device. KEY must
match the KEY variable in this sketch to successfully unlock.

UNLOCK SEQUENCE:
e2b.reset();
e2b.skip();         //DO NOT USE e2b.select(addr);
e2b.unlock(KEY);

e2b.reset();
e2b.skip();         //DO NOT USE e2b.select(addr);
e2b.write(data);

*/
#define E2B_pin 2

//unsigned char rom[8];
unsigned char rom[8] = {FAMILYCODE, 0xE2, 0xCC, 0x2D, 0x01, 0x25, 0xB8, 0x30};
unsigned char scratchpad[9] = {0x5E, 0x00, 0x00, 0x04, 0x00, 0x31, 0x6A, 0xD7, 0x00};

uint8_t KEY = 0x30;

E2B e2b(E2B_pin);  // on pin 2 (a 4.7K resistor is necessary)

void setup() {
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial){}
  Serial.println("E2B Secured Slave Node Test.");
  //randomSeed(analogRead(0));          //Uncomment when generating new rom address
  //e2b.generateROM(rom);               //Uncomment when generating new rom address
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);

  e2b.setSecureFlag(1,KEY);             //Sets the device as a secure node with the unlock key as KEY

  //e2b.setDeviceType(POINTTOPOINT);    //Can use this when only one device is connected
}

void respond(){
  e2b.MasterResetPulseDetection();
}

void loop(void) {
  e2b.waitForRequest(false);
  Serial.println(e2b.getScratchpad(4),HEX);
}
