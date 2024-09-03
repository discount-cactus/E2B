//E2B Scanner
//pinScan() scans a single pin for devices
//deviceScan() scans every pin in the specified range for devices
#include <E2B.h>

const int startPin = 2;
const int endPin = 20;

unsigned char rom[8] = {FAMILYCODE,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

int count = 0;

void setup(){
  Serial.begin(9600);
  Serial.println("Starting E2B Search.");
  //pinScan(startPin);
  deviceScan(startPin,endPin);
  Serial.println("E2B Search complete.");
}

void loop(){
}



int pinScan(int pin){
  E2B e2b(pin);
  //e2b.init(rom);
  int num = 0;

  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  //byte sl[8];                                   //Uncomment to use search_and_log
  //byte searchlog[100][8];                       //Uncomment to use search_and_log
  
  while(e2b.search(addr)){
  //while(e2b.search_and_log(addr,sl)){          //Uncomment to use search_and_log
    Serial.print("Address found on Pin ");
    Serial.print(pin);
    Serial.print(":\t");
    //Address
    for(i=0; i < 8; i++){
      Serial.print("0x");
      if (addr[i] < 16){
        Serial.print('0');
      }
      Serial.print(addr[i], HEX);
      if (i < 7) {
        Serial.print(" ");
      }
      //searchlog[count][i] = sl[i];          //Uncomment to use search_and_log
    }
    
    //Device type
    switch(addr[0]){
      default:
        Serial.print("\t\tUnknown");
        break;
      case FAMILYCODE_HOST:
        Serial.print("\t\tHOST");
        break;
       case FAMILYCODE_TRANSCEIVER:
        Serial.print("\t\tTRANSCEIVER");
        break;
      case 0x10:
        Serial.print("\t\tDS18S20");  // or old DS1820
        break;
      case 0x18:
        Serial.print("\t\tDS2484");
        break;
      case 0x1D:
        Serial.print("\t\tDS2423");
        break;
      case 0x22:
        Serial.print("\t\tDS1822");
        break;
      case 0x27:
        Serial.print("\t\tDS2417");
        break;
      case 0x28:
        Serial.print("\t\tDS18B20");
        break;
      case 0x29:
        Serial.print("\t\tDS2408");
        break;
      case 0x56:
        Serial.print("\t\tDS28E18");
        break;
      case 0xA0:
        Serial.print("\t\tATtiny");
        break;
      case 0xA1:
        Serial.print("\t\tArduino");
        break;
      case 0xA7:
        Serial.print("\t\tESP32");
        break;
      case 0xA8:
        Serial.print("\t\tSTM32");
        break;
      case 0xA9:
        Serial.print("\t\tSAMD21");
        break;
    }
    
    //CRC check
    if (E2B::crc8(addr,7) == addr[7]){
      Serial.print("\t\t CRC verified");
    }else{
      Serial.print("\t\t CRC failed");
    }

    //Searchlog check
    /*Serial.print("\t\t");
    for(i=0; i < 8; i++){
      Serial.print("0x");
      if (searchlog[count][i] < 16){
        Serial.print('0');
      }
      Serial.print(searchlog[count][i], HEX);
      if (i < 7){
        Serial.print(" ");
      }
    }*/
    count++;
    num++;
    Serial.println();
  }
  e2b.reset_search();
  
  if(num){
    Serial.print("Total devices found on pin " + String(pin) + ": ");
    Serial.println(num);
  }
  return num;
}

int deviceScan(int startPin, int endPin){
  //int count = 0;
  for(uint8_t pin=startPin; pin < endPin; pin++){
    pinScan(pin);
  }
  if(count){
    Serial.print("Total devices found: ");
    Serial.println(count);
  }
  return count;
}
