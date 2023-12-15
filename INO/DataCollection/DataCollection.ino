#include <ACS712.h>
#include <ZMPT101B.h>

#define SENSITIVITY 500.0f

ZMPT101B voltageSensor(4, 50.0);
ACS712  ACS(1,3.3, 4095, 185);

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  voltageSensor.setSensitivity(SENSITIVITY);
  ACS.autoMidPoint();
  //Serial.print("MidPoint: ");
  //Serial.println(ACS.getMidPoint());
  //Serial.print("Noise mV: ");
  //Serial.println(ACS.getNoisemV());
}

void loop() {
    unsigned long currentTime = millis();
    ACS.suppressNoise(true);
    float average = 0;
    for (int i = 0; i < 10; i++)
    {
      average += ACS.mA_AC();
    }
    float mA = average /10.0;
    float volt = voltageSensor.getRmsVoltage();
    
    //Serial.print("Current = ");
    Serial.print(mA);
    Serial.print(",");
    //Serial.print("Voltage =");
    Serial.println(volt);
}
