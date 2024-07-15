//ATtiny UART-to-E2B bridge example
//Simple conversion from UART-to-E2B bridge for use as a seamless transceiver between devices

/*NOTES:
-MADE FOR USE WITH THE ATTINY85 MICROCONTROLLER
-Receives a UART packet and transmits it on the E2B pin
-Receives a E2B packet and transmits it on the UART pins

-ATtiny85 does not feature a dedicated UART port, so SoftwareSerial is used

-TaskScheduler is used to complete sending and receiving of different protocols in parallel
-Link to TaskScheduler library: https://github.com/arkhipenko/TaskScheduler

Hookup :
ATtiny85 RX pin: -> Arduino RX     (pin 0 on Arduino Uno)
ATtiny85 TX pin: -> Arduino TX     (pin 1 on Arduino Uno)

*/
#include <TaskScheduler.h>
#include <E2B.h>
#include <SoftwareSerial.h>
 
#define E2B_pin 2
#define rxPin 10 //3
#define txPin 11 //4

unsigned char rom[8] = {FAMILYCODE, 0xB2, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte localscratchpad[9]; // buffer for data
//byte addr[8]; // 64-bit device address

E2B e2b(E2B_pin);
SoftwareSerial mySerial(rxPin, txPin);

// Callback methods prototypes
void t1Callback();
void t2Callback();

//Tasks
Task t1(TASK_IMMEDIATE, TASK_FOREVER, &t1Callback);
Task t2(TASK_IMMEDIATE, TASK_FOREVER, &t2Callback);

Scheduler runner;


void t1Callback(){
  //Get UART, send E2B
  if (mySerial.available()){
    uint8_t dataUART = mySerial.read();
    Serial.print(dataUART,HEX);
    
    boolean present;
    present = e2b.reset();                // device present var
    e2b.skip(); 
    if(present){
      e2b.write(dataUART);
    }
  }
}

void t2Callback() {
  //Get E2B, send UART
  e2b.waitForRequest(false);
  uint8_t dataE2B = e2b.scratchpad[4];
  mySerial.write(dataE2B);
  Serial.println(dataE2B,HEX);
}

void setup(){
  runner.init();
  runner.addTask(t1);
  runner.addTask(t2);
  t1.enable();
  //t2.enable();

  attachInterrupt(E2B_pin,respond,CHANGE);
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);
  Serial.begin(9600);
  while(!Serial){}
  mySerial.begin(9600);
}

void respond(){
  e2b.MasterResetPulseDetection();
}

void loop(){
  runner.execute();
}
