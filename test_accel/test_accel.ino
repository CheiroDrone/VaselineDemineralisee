#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Ultrasonic.h>

#include <Wire.h>

Ultrasonic ultrasonic(6);

FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
int16_t angle[2]; // pitch & roll

int vitesseMoteur[4]; // from 0 to 180
int pourcentageX[4]; // from -5 to 5
int pourcentageY[4]; // from -5 to 5
int pourcentage[4];

// Moteurs
// [0] [1]
// [2] [3]

void updateMotorSpeedFromX(const double x)
{
  const int delta = map(x, -90, 90, -10, 10);
  pourcentageX[0] += delta;
  pourcentageX[2] += delta;
  pourcentageX[1] -= delta;
  pourcentageX[3] -= delta;
}

void updateMotorSpeedFromY(const double y)
{
  const int delta = map(y, -90, 90, -10, 10);
  pourcentageY[0] -= delta;
  pourcentageY[1] -= delta;
  pourcentageY[2] += delta;
  pourcentageY[3] += delta;
}

void updateMotorSpeed(double x, double y)
{
  for (int i = 0; i < 4; ++i)
  {
    pourcentageX[i] = 100;
    pourcentageY[i] = 100;
  }

  updateMotorSpeedFromX(x);
  updateMotorSpeedFromY(y);

  for (int i = 0; i < 4; ++i)
  {
    pourcentage[i] = (pourcentageX[i] + pourcentageY[i]) / 2;
  }

  for (int i = 0; i < 4; ++i)
  {
    Serial.print("  M");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(pourcentage[i]);
  }
  Serial.println();
}

void setup()
{
  Serial.begin(19200);
  Wire.begin();
  sixDOF.init();
}

void loop()
{
  sixDOF.getRawValues(rawSixDof);
  angle[0] = _atan2(rawSixDof[0],rawSixDof[2]);
  angle[1] = _atan2(rawSixDof[1],rawSixDof[2]);
  Serial.print(" X: ");
  Serial.print(angle[0]/10.0);

  Serial.print(" Y: ");
  Serial.print(angle[1]/10.0);
  Serial.println();

  updateMotorSpeed(angle[0] / 10.0, angle[1] / 10.0);

  long distance = ultrasonic.MeasureInCentimeters();
  Serial.println(distance);

  delay(100);
}

int16_t _atan2(int32_t y, int32_t x)
{
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) )
  {
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x<0)
    {
    if (y<0) a -= 1800;
    else a += 1800;
    }
  }
  else
  {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y<0) a -= 1800;
  }
  return a;
}


