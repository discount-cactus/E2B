//DS24B33 4kbit EEPROM Example
//Write + Read
#include <E2B.h>

#define E2B_pin 2

E2B e2b(E2B_pin);

void setup(){
  Serial.begin(9600);
  while(!Serial){}

  Serial.println("DS24B33 4kbit EEPROM Write + Read Example");
}

void loop(){
  byte addr[8];
  byte data[32];
  byte i;

  // Fill buffer with test data (0x00 to 0x1F)
  for(i=0; i < 32; i++){
    data[i] = i;
  }

  // Find DS24B33 device
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

  if(addr[0] != 0x23){
    Serial.println("Device is not a DS24B33.");
    return;
  }

  // ---- WRITE TO EEPROM ----
  uint16_t targetAddr = 0x0000;

  // Step 1: Write to Scratchpad
  e2b.reset();
  e2b.select(addr);
  e2b.write(0x0F); // Write Scratchpad
  e2b.write(lowByte(targetAddr));  // TA1 (LSB)
  e2b.write(highByte(targetAddr)); // TA2 (MSB)

  for(i=0; i < 32; i++){
    e2b.write(data[i]);
  }

  // Step 2: Read Scratchpad (optional, to verify address and data)
  e2b.reset();
  e2b.select(addr);
  e2b.write(0xAA); // Read Scratchpad

  byte ES; // Ending offset
  byte scratchpad[35]; // TA1, TA2, ES + 32 bytes + CRC
  for(i=0; i < 35; i++){
    scratchpad[i] = e2b.read();
  }

  ES = scratchpad[2];
  Serial.print("Scratchpad ending offset: ");
  Serial.println(ES);

  // Step 3: Copy Scratchpad to EEPROM
  e2b.reset();
  e2b.select(addr);
  e2b.write(0x55); // Copy Scratchpad
  e2b.write(lowByte(targetAddr));  // TA1
  e2b.write(highByte(targetAddr)); // TA2
  e2b.write(ES);                   // Ending offset

  delay(20); // EEPROM write cycle time (tPROG = 10 ms max, use 20 for safety)

  //Reads from EEPROM
  e2b.reset();
  e2b.select(addr);
  e2b.write(0xF0);              // Read Memory
  e2b.write(lowByte(targetAddr));  // LSB address
  e2b.write(highByte(targetAddr)); // MSB address

  Serial.print("Readback Data at 0x0000: ");
  for(i=0; i < 32; i++){
    byte val = e2b.read();
    Serial.print(val, HEX);
    Serial.print(" ");
  }
  Serial.println();

  delay(3000);
}
