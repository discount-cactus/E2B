//UART-to-E2B Bridge example
// Simple conversion from UART-to-E2B bridge for use as a seamless transceiver between devices that
// may not support direct compatibility to an E2B bus

/* NOTES:
 *  Uses the "Tasks" library by Ethan McTague. Link: https://github.com/emctague/Tasks
 * Hookup:
 * ATTINY85 pin 3 -> Arduino Uno pin 0
 * ATTINY85 pin 4 -> Arduino Uno pin 1
*/


#include <E2B.h>
#include <SoftwareSerial.h>
#include <Tasks.h>
 
#define E2B_pin 2
#define RXpin 3
#define TXpin 4

unsigned char rom[8] = {FAMILYCODE, 0xB2, 0xDD, 0x03, 0x00, 0x00, 0x11, 0x00};
unsigned char scratchpad[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte localscratchpad[9]; // buffer for data
byte addr[8]; // 64-bit device address

E2B e2b(E2B_pin);
SoftwareSerial mySerial(RXpin,TXpin);


// scheduler tasks
//Task 1: Receives E2B message, transmits message on UART
void task_1(void){
  e2b.waitForRequest(false);
  byte dataE2B = e2b.scratchpad[0];
  mySerial.print(dataE2B);
}

//Task 2: Receives UART message, transmits message on E2B
void task_2(void){
  while(mySerial.available()){
    byte dataUART = mySerial.read();
    //e2b.reset();
    //e2b.select(addr);
    e2b.write(dataUART);
  }
}


void setup(){
  //Initializes E2B bus
  attachInterrupt(E2B_pin,respond,CHANGE);
  e2b.init(rom);
  e2b.setScratchpad(scratchpad);

  //Initializes SoftwareSerial lines
  mySerial.begin(9600);
  
  //Init task scheduler
  Tasks_Init();

  // print delay between calls to serial console
  Tasks_Add((Task) task_1, 10, 0);
  Tasks_Add((Task) task_2, 10, 100);
  
  //Start task scheduler
  Tasks_Start();
}

void respond(){
  e2b.MasterResetPulseDetection();
}

void loop(){
}
