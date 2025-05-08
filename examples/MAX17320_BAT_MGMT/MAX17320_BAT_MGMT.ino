//E2B MAX17320 Battery Management IC example
/*NOTES:
FAMILY CODE: TBD (potentially 0x01)

*/
#include <E2B.h>

#define E2B_PIN 2

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Useful commands to send for reading/writing to the device, refer to datasheet for specific use
//Capacity configuration
#define RepCap 0x05         //Remaining capacity (mAh, 0.5mAh/LSB), used for SOC calculation, read-only
#define FullCapRep 0x1A     //Full capacity the gauge uses to calculate SOC, useful for calibration
#define DesignCap 0x18      //Design capacity of battery (in mAh), set during initialization
#define FullCapNom	0x1C    //Nominal capacity, can also be used for SOC reference

//Power/Charging configuration
#define VCELL 0x0D          //Battery Voltage (78.125uV/LSB),	read-only
#define Current	0x0A        //Current draw/input (signed, 1.5625uV/RSense per LSB), read-only
#define	AvgCurrent 0x04     //Smoothed average current, read-only
#define AvgPower 0x09       //Computed average power, read-only

//Environmental configuration
#define Temperature 0x08    //Raw temperature (Kelvin, 1/256 K/LSB)
#define TTE 0x1E            //Time to empty (minutes)
#define TTF 0x1D            //Time to full (minutes)

//Operational settings (USE WITH CAUTION)
#define Config 0x1F         //Bitfield for feature enable/disable,	advanced use only
#define LearnConfig 0x60    //Learning mode config,	advanced
#define HibCfg 0xBB         //Hibernate mode thresholds, power saving config
#define PackCfg 0xB3        //Pack-level configuration
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

E2B max17320(E2B_PIN);

void setup(){
  Serial.begin(9600);
  while(!Serial){}
  Serial.println("MAX17320 Battery Management IC example");
}

void loop(){
  byte addr[8];
  
  if(!max17320.search(addr)) {
    Serial.println("No more devices found.");
    max17320.reset_search();
    delay(1000);
    return;
  }

  Serial.print("Found device ROM: ");
  for(int i=0; i < 8; i++) {
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if(E2B::crc8(addr,7) != addr[7]) {
    Serial.println("\nCRC is not valid!");
    return;
  }

  //Check if it might be a MAX17320 (based on family code)
  if (addr[0] == 0x01){
    Serial.println("\nMAX17320 device detected!");

    // Read and display several registers
    readRegister(addr, 0x0D, "VCELL (Voltage)", 0.078125 / 1000.0, "V");
    readRegister(addr, 0x0A, "CURRENT", 1.5625 / 1000.0, "A");              // Signed
    readRegister(addr, 0x08, "TEMPERATURE", 1.0 / 256.0, "°C");             // Kelvin to Celsius
    readRegister(addr, 0x05, "REMAINING CAPACITY", 0.5, "mAh");

    computeSOC(addr);
    dumpRegisters(addr);

    //Optional examples of writing to registers
    //writeRegister(addr, 0x1A, 3000);   //Set FullCapRep to 3000mAh
    //writeRegister(addr, 0x18, 6000);      //Set DesignCap to 3000 mAh (in mAh, *2 = 6000 because it's 0.5mAh/LSB)
    //writeRegister(addr, 0x1A, 6000);      //Set FullCapRep to match
  }else{
    Serial.println("\nNot a recognized MAX17320 device.");
    return;
  }
  delay(3000);
}

// Helper function to read 2-byte register and print scaled value
void readRegister(byte *romAddr, byte reg, const char* label, float scale, const char* unit) {
  if (!max17320.reset()) {
    Serial.println("Device not responding.");
    return;
  }

  max17320.select(romAddr);
  max17320.write(0x69); // READ MEMORY command
  max17320.write(reg);  // Register address
  max17320.write(0x00); // Page number (usually 0)

  byte lsb = max17320.read();
  byte msb = max17320.read();
  int16_t raw = (msb << 8) | lsb;

  float value = raw * scale;

  // Convert temperature from Kelvin if needed
  if (strcmp(label, "TEMPERATURE") == 0) {
    value -= 273.15;
  }

  Serial.print("  ");
  Serial.print(label);
  Serial.print(": ");
  Serial.print(value, 3);
  Serial.print(" ");
  Serial.println(unit);
}

// Helper to read raw register value
int16_t readRegisterRaw(byte *romAddr, byte reg) {
  if (!max17320.reset()) {
    Serial.println("Device not responding.");
    return -1;
  }

  max17320.select(romAddr);
  max17320.write(0x69); // READ MEMORY
  max17320.write(reg);
  max17320.write(0x00); // Page 0

  byte lsb = max17320.read();
  byte msb = max17320.read();
  return (int16_t)((msb << 8) | lsb);
}

// Simple function to write a 16-bit value to a register
void writeRegister(byte *romAddr, byte reg, uint16_t value) {
  if (!max17320.reset()) {
    Serial.println("Device not responding.");
    return;
  }

  max17320.select(romAddr);
  max17320.write(0x6C); // WRITE MEMORY command
  max17320.write(reg);  // Register address
  max17320.write(0x00); // Page number (0 for most cases)

  max17320.write((byte)(value & 0xFF));       // LSB
  max17320.write((byte)((value >> 8) & 0xFF)); // MSB

  Serial.print("Wrote 0x");
  Serial.print(value, HEX);
  Serial.print(" to register 0x");
  Serial.println(reg, HEX);
}

//Computes the state-of-charge (SOC) from RepCap and FullCapRep
void computeSOC(byte *romAddr) {
  int16_t repCap = readRegisterRaw(romAddr, 0x05);  // RepCap
  int16_t fullCap = readRegisterRaw(romAddr, 0x1A); // FullCapRep

  if (fullCap <= 0) {
    Serial.println("Invalid FullCap, cannot compute SOC.");
    return;
  }

  float soc = (float)repCap / fullCap * 100.0;

  Serial.print("  State of Charge (SOC): ");
  Serial.print(soc, 2);
  Serial.println(" %");
}

// Reads and prints a group of key MAX17320 registers for debugging
void dumpRegisters(byte *romAddr) {
  struct RegInfo {
    byte reg;
    const char* name;
    float scale;
    const char* unit;
    bool signedVal;
  };

  RegInfo regs[] = {
    {0x0D, "VCELL", 0.078125 / 1000.0, "V", false},
    {0x0A, "CURRENT", 1.5625 / 1000.0, "A", true},
    {0x08, "TEMP", 1.0 / 256.0, "°C", false},
    {0x05, "REPCAP", 0.5, "mAh", false},
    {0x18, "DESIGNCAP", 0.5, "mAh", false},
    {0x1A, "FULLCAPREP", 0.5, "mAh", false},
    {0x1C, "FULLCAPNOM", 0.5, "mAh", false},
    {0x06, "REPSOC", 1.0, "%", false},
    {0x07, "MIXSOC", 1.0, "%", false},
    {0x1E, "TTE", 1.0, "min", false},
    {0x1D, "TTF", 1.0, "min", false},
  };

  Serial.println("Register Dump:");
  for (unsigned int i = 0; i < sizeof(regs)/sizeof(RegInfo); i++) {
    int16_t raw = readRegisterRaw(romAddr, regs[i].reg);

    float value = regs[i].signedVal ? (int16_t)raw * regs[i].scale : (uint16_t)raw * regs[i].scale;

    // Convert Kelvin to Celsius if it's temperature
    if (strcmp(regs[i].name, "TEMP") == 0) value -= 273.15;

    Serial.print("  ");
    Serial.print(regs[i].name);
    Serial.print(" = ");
    Serial.print(value, 3);
    Serial.print(" ");
    Serial.println(regs[i].unit);
  }
}
