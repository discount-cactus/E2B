//DS2408 8-Channel I/O Expander
/*Notes:
 * -DS2408 doesn't have mode bits to set pins to input or output.
 *      If you issue a read command, they're inputs.
 *      If you write to them, they're outputs.
 * -For reading from a switch, you should use 10K pull-up resisters.
 */
#include <E2B.h>

#define E2B_pin 2

unsigned char rom[8] = {FAMILYCODE, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x4B, 0x46, 0x7F, 0xFF, 0x00, 0x10, 0x00};

E2B ds(E2B_pin);  // on pin 2 (a 4.7K resistor is necessary)

byte addr[8];

void PrintBytes(const uint8_t* addr, uint8_t count, bool newline=false){
  for (uint8_t i=0; i < count; i++){
    Serial.print("0x");
    Serial.print(addr[i]>>4, HEX);
    Serial.print(addr[i]&0x0f, HEX);
    if(i < count-1){
      Serial.print(", ");
    }
  }
  if(newline){
    Serial.println();
  }
}

void setup(){
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

void loop(){
  //Searches for a compatible (DSD2408) device
  while(!ds.search(addr)){
    Serial.print("No more addresses.\n");
    ds.reset_search();
    delay(1000);
    return;
  }

  //Verifies device is a DS2408
  if(addr[0] != 0x29){
    PrintBytes(addr,8);
    Serial.print(" is not a DS2408.\n");
    return;
  }

  //Verifies CRC
  if(E2B::crc8(addr,7) != addr[7]){
    Serial.println("CRC invalid.");
    return;
  }
  
  Serial.print("  Reading DS2408 ");
  PrintBytes(addr,8,true);
  //Serial.println();

  uint8_t buf[13];  // Put everything in the buffer so we can compute CRC easily.
  buf[0] = 0xF0;    // Read PIO Registers
  buf[1] = 0x88;    // LSB address
  buf[2] = 0x00;    // MSB address
  ds.write_bytes(buf, 3);
  ds.read_bytes(buf+3, 10);     // 3 cmd bytes, 6 data bytes, 2 0xFF, 2 CRC16
  ds.reset();

  //Verifies (16-bit) CRC
  if (!E2B::check_crc16(buf, 11, &buf[11])){
    Serial.print("CRC failure in DS2408 at ");
    PrintBytes(addr,8,true);
    return;
  }
  Serial.print("  DS2408 data = ");
  Serial.println(buf[3],BIN);      // First 3 bytes contain command, register address.
}
