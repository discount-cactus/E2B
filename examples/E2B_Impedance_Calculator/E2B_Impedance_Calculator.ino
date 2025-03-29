//E2B Impedance Calculators
//Useful in-library tools for designing E2B networks
/*NOTES:
Microstrip - Trace on external layers
Stripline - Trace on internal layers
*/

const float dielectricConstant = 4.6;   //Standard value for FR4

void setup(){
  Serial.begin(9600);
  while(!Serial);
  Serial.println("E2B Impedance Calculator.");

  float Zo = getImpedanceMicrostrip(1.5,0.035,0.79375,dielectricConstant);
  //float Zo = getImpedanceStripline(1.5,0.035,0.79375/2,dielectricConstant);
  float Co = getCapacitanceMicrostrip(1.5,0.035,0.79375,dielectricConstant);
  float Lo = getInductance(Zo,Co);

  Serial.print("Microstrip Impedance: "); Serial.println(Zo);
  Serial.print("Stripline Impedance: "); Serial.println(Zo);
  Serial.print("Microstrip Capacitance: "); Serial.println(Co,15);
  Serial.print("Microstrip Inductance: "); Serial.println(Lo);
}

void loop(){
}

//Calculates characteristic impedance of a microstrip based on Waddell's equations
float getImpedanceMicrostrip(float trackWidth, float trackThickness, float dielectricThickness, float er_in){
  float  pi = 3.1415;
  float W_;
  float Eeff;
  float Zo;
  float tWidth = trackWidth;                // in millimeters
  float tThickness = trackThickness;        // in millimeters
  float dThickness = dielectricThickness;   // in millimeters
  float er = er_in;

  Eeff = 1/sqrt(1+((12*dThickness)/tWidth));
  if ( (tWidth/dThickness) < 1 ){
    Eeff += 0.04 + pow(1 - (tWidth/dThickness),2);
  }
  Eeff *= (er - 1)/2;
  Eeff += (er + 1)/2;

  W_ = pow(tThickness/dThickness,2);
  W_ += pow( (1/pi) / ((tWidth/tThickness) + 1.1) ,2);
  W_ = (4*exp(1)) / W_;
  W_ = log(W_);
  W_ *= tThickness/pi;
  W_ *= (1+pow(er,-1))/2;
  W_ += tWidth;

  Zo = (14 + (8/er)) / 11;
  Zo = Zo * ((4*dThickness) / W_);
  Zo = Zo + sqrt(pow(Zo,2) + (pow(pi,2) * ((1+pow(er,-1))/2) ));
  Zo *= (4*dThickness) / W_;
  Zo += 1;
  Zo = log(Zo);
  Zo *= 60/sqrt((2*er)+2);

  return Zo;
}

//Calculates characteristic impedance of a stripline based on Waddell's equations
float getImpedanceStripline(float trackWidth, float trackThickness, float dielectricThickness, float er_in){
  const float  pi = 3.1415;
  float Weff;
  float Zo;
  float tWidth = trackWidth;                // in millimeters
  float tThickness = trackThickness;        // in millimeters
  float dThickness = dielectricThickness;   // in millimeters
  float er = er_in;

  float m = (6 * dThickness) / ((3 * dThickness) + tThickness);

  Weff = pow(tThickness / ((4 * dThickness) + tThickness),2);
  Weff += pow( (pi * tThickness) / (4 * (tWidth + (1.1 * tThickness) ) ),m);
  Weff = sqrt(Weff);
  Weff = er / Weff;
  Weff = log(Weff);
  Weff *= tThickness / pi;
  Weff += tWidth;

  Zo = (16 * dThickness) / (pi * Weff);
  Zo = Zo + sqrt(pow(Zo,2) + 6.27);
  Zo *= (8 * dThickness) / (pi * Weff);
  Zo += 1;
  Zo = log(Zo);
  Zo *= 60/sqrt(er);

  return Zo;
}

//Calculates capacitance of a trace
float getCapacitanceMicrostrip(float trackWidth, float trackThickness, float dielectricThickness, float er_in){
  float Co;
  float tWidth = trackWidth;                // in millimeters
  float tThickness = trackThickness;        // in millimeters
  float dThickness = dielectricThickness;   // in millimeters
  float er = er_in;

  Co = 2.64 * pow(10,-11);
  Co *= er + 1.41;
  Co /= log((5.98 * dThickness) / ((0.8*tWidth) + tThickness));

  return Co;
}

float getCapacitanceStripline(float trackWidth, float trackThickness, float dielectricThickness, float er_in){
  float Co;
  float tWidth = trackWidth;                // in millimeters
  float tThickness = trackThickness;        // in millimeters
  float dThickness = dielectricThickness;   // in millimeters
  float er = er_in;

  Co = 5.55 * pow(10,-11);
  Co *= er;
  Co /= log((3.81 * dThickness) / ((0.8*tWidth) + tThickness));

  return Co;
}

//Calculates characteristic inductance of a trace from its characteristic impedance and getCapacitanceStripline
// From Zo = sqrt(L/C);
float getInductance(float Zo, float Co){
  float Lo = Co * Zo * Zo;

  return Lo;
}