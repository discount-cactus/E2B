//E2B DS2417 RTC Example
//Code is largely from the forum: https://forum.arduino.cc/t/adding-ds2417-rtc-to-existing-analog-clock-code/556083/3
#include <E2B.h>

#define E2B_pin 2

unsigned char rom[8] = {FAMILYCODE, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

E2B e2b(E2B_pin);

void setup(void){
  Serial.begin(9600);
  while(!Serial){}
}

void loop(){
  byte i;
  byte present = 0;
  byte data[8];
  byte addr[8];

  if (!e2b.search(addr)){
      Serial.print("No more addresses found.\n");
      e2b.reset_search();
      delay(500);  // for readability
      return;
  }

  Serial.print("ROM: ");
  for(i=0; i < 8; i++){
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if(E2B::crc8(addr,7) != addr[7]){
      Serial.print("CRC is not valid!\n");
      return;
  }

  if(addr[0] != 0x27){
      Serial.print("\t\tDevice is not a DS1904 family device.\n");
      return;
  }


  //Write to the device
  Serial.println("writing to RTC...");
  present = e2b.reset();
  e2b.select(addr);
  e2b.write(0x99,1);   // write RTC - this is the write code
  e2b.write(0xAC);  //This is the control byte.  AC in hex = 10101100
                   //read the datasheet and you will see that this is important
                   //to start the internal osc's... Or to make the clock start
                   //counting secone2b.  --ril3y
 // e2b.write(0x00);  //0x02 is a random time set it with your own
 // e2b.write(0x03);  //same with this
//  e2b.write(0x05);  //this
 // e2b.write(0x08);  //and this
  present = e2b.reset();
  delay(1000);     // unknown if wait needed


  //Read from device
  present = e2b.reset();
  e2b.select(addr);
  e2b.write(0x66,1);   // read RTC

  Serial.print("PR: ");
  Serial.print(present, HEX);
  for (i=0; i < 5; i++) {
    data[i] = e2b.read();
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
