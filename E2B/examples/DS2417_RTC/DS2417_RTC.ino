//E2B DS2417 RTC example
#include <E2B.h>

#define E2B_pin 2

unsigned char rom[8] = {FAMILYCODE, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

E2B ds(E2B_pin);

void setup(void) {
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  ds.init(rom);
  ds.setScratchpad(scratchpad);
}

void respond(){
  ds.MasterResetPulseDetection();
}

void loop(){
  byte i;
  byte present = 0;
  byte data[8];
  byte addr[8];

  if (!ds.search(addr)){
      Serial.print("No more addresses found.\n");
      ds.reset_search();
      delay(500);  // for readability
      return;
  }

  Serial.print("ROM: ");
  for(i=0; i < 8; i++){
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if(E2B::crc8( addr, 7) != addr[7]){
      Serial.print("CRC is not valid!\n");
      return;
  }

  if(addr[0] != 0x27){
      Serial.print("\t\tDevice is not a DS1904 family device.\n");
      return;
  }


  // write!
  Serial.println("writing to RTC...");
  present = ds.reset();
  ds.select(addr);
  ds.write(0x99,1);   // write RTC - this is the write code
  ds.write(0xAC);  //This is the control byte.  AC in hex = 10101100
                   //read the datasheet and you will see that this is important
                   //to start the internal osc's... Or to make the clock start
                   //counting seconds.  --ril3y
 // ds.write(0x00);  //0x02 is a random time set it with your own
 // ds.write(0x03);  //same with this
//  ds.write(0x05);  //this
 // ds.write(0x08);  //and this
  present = ds.reset();
  delay(1000);     // unknown if wait needed


  // read!
  present = ds.reset();
  ds.select(addr);
  ds.write(0x66,1);   // read RTC

  Serial.print("PR: ");
  Serial.print(present, HEX);
  for (i=0; i < 5; i++) {
    data[i] = ds.read();
  }
  Serial.print(" CTRL BYTE:  ");
  Serial.print(data[0], BIN);
  Serial.print("\n\ttime:  ");
  for (i=1; i < 5; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
    }


  Serial.println();
}
