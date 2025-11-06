//DS2502 1kbit EPROM Example
/*NOTES:
-Only 1 to 0 transitions are allowed. If you try to write a 1 to a bit already 0,
 the data won't change—and you'll never get that 1 back.
*/
#include <E2B.h>

#define E2B_pin 2

E2B e2b(E2B_pin);

void setup(){
  Serial.begin(9600);
  while(!Serial){}

  Serial.println("DS2502 1kbit EPROM Write + Read Example");
}

void loop(){
  byte addr[8];
  byte i;
  uint8_t data[32];

  // Fill buffer with sample data (0x00 to 0x1F)
  for (i=0; i < 32; i++){
    data[i] = i;
  }

  // Search for DS2502
  if(!e2b.search(addr)){
    Serial.println("No more addresses.");
    e2b.reset_search();
    delay(1000);
    return;
  }

  Serial.print("ROM = ");
  for(i=0; i < 8; i++){
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if(E2B::crc8(addr, 7) != addr[7]){
    Serial.println("CRC is not valid!");
    return;
  }

  if(addr[0] != 0x09) {
    Serial.println("Device is not a DS2502.");
    return;
  }

  uint16_t targetAddr = 0x0000;

  //Writes to DS2502 memory
  //WARNING: DS2502 supports writing only from 1 to 0 (one-time programmable)
  /*Serial.println("Writing to DS2502...");
  for(i=0; i < 32; i++){
    e2b.reset();
    e2b.select(addr);
    e2b.write(0x0F); // Write Memory
    e2b.write(lowByte(targetAddr + i));  // TA1
    e2b.write(highByte(targetAddr + i)); // TA2
    e2b.write(data[i]);

    delay(20);  // tPROG for EPROM write (max ~10ms, use 20ms for margin)
  }*/

  //Reads from DS2502 memory
  Serial.println("Reading from DS2502...");
  e2b.reset();
  e2b.select(addr);
  e2b.write(0xF0);  // Read Memory
  e2b.write(lowByte(targetAddr));  // TA1
  e2b.write(highByte(targetAddr)); // TA2

  Serial.print("Data at 0x0000: ");
  for(i=0; i < 32; i++){
    uint8_t val = e2b.read();
    Serial.print(val, HEX);
    Serial.print(" ");
  }
  Serial.println();

  delay(3000);
}
