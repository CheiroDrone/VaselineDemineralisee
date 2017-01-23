#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Ultrasonic.h>

#include <Wire.h>

Ultrasonic ultrasonic(7);

FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
int16_t angle[2]; // pitch & roll

static int baseSpeed;
static int vitesseMoteur[4] = {}; // from 0 to 180
static int pourcentageX[4] = {}; // 100% + ou - 10%
static int pourcentageY[4] = {}; // 100% + ou - 10%
static int pourcentage[4] = {}; // 100% + ou - 10%
static long altitude;
static long delta_altitude;

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

// à changer
int getMotorBaseSpeed()
{
  int vitesse = map(delta_altitude, -10, 10, 0, 180);
  return 0;
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

void updateAltitude()
{
  long new_altitude = ultrasonic.MeasureInCentimeters();
  delta_altitude = altitude - new_altitude;
  if (new_altitude >= 300 || abs(delta_altitude) > 10)
    return;
  altitude = new_altitude;
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

  updateAltitude();
  updateMotorSpeed(angle[0] / 10.0, angle[1] / 10.0);
  
  Serial.println(altitude);

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


