//E2B Impedance Calculators
#include <E2B.h>

#define E2B_pin 2

unsigned char rom[8] = {FAMILYCODE, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x11, 0x00};

E2B e2b(E2B_pin);  // on pin 2 (a 4.7K resistor is necessary)

void setup(){
  attachInterrupt(E2B_pin,respond,CHANGE);
  Serial.begin(9600);
  while(!Serial);
  Serial.println("E2B Impedance Calculator.");

  float Zo = e2b.getImpedanceMicrostrip(1.5,0.035,0.79375);
  //float Zo = e2b.getImpedanceStripline(1.5,0.035,0.79375/2);
  //float Co = e2b.getCapacitanceMicrostrip(1.5,0.035,0.79375);
  //float Lo = e2b.getInductance(Zo,Co);

  Serial.print("Microstrip Impedance: "); Serial.println(Zo);
  //Serial.print("Stripline Impedance: "); Serial.println(Zo);
  //Serial.print("Microstrip Capacitance: "); Serial.println(Co,15);
  //Serial.print("Microstrip Inductance: "); Serial.println(Lo);
}

void respond(){
  e2b.MasterResetPulseDetection();
}

void loop(){
}
