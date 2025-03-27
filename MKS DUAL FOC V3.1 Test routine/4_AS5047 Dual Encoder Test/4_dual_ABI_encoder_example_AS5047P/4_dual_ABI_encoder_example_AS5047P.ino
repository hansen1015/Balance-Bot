// MKS DUAL FOC ABI encoder test routine.Test hardware: MKS DUAL FOC V3.1
// Encoder for testing AS5047P,CPR=4000


#include <SimpleFOC.h>

//For ABI interface (0) number
//Encoder encoder = Encoder(19,18,1000,15); //A0;B0;Encoder PPR,PPR=CPR/4;I0

//For ABI interface (1) number
Encoder encoder = Encoder(23,5,1000,13); //A1;B1;Encoder PPR,PPR=CPR/4;I1

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {
  Serial.begin(115200);
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_EXTERN;

  encoder.init();
  // Hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop() {
  // Output Angle and Angular Velocity
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}
