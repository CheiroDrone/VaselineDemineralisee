#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Ultrasonic.h>
#include <Servo.h>

#include <Wire.h>

Ultrasonic ultrasonic(7);

FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
int16_t angle[2]; // pitch & roll

static int vitesseMoteur[4] = {}; // from 0 to 180
static int pourcentageX[4] = {}; // 100% + ou - 10%
static int pourcentageY[4] = {}; // 100% + ou - 10%
static int pourcentage[4] = {}; // 100% + ou - 10%
static long altitude;
static long delta_altitude;
static int zero = 0;

Servo moteur[4];

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
//  if (altitude - zero < 5)
//    return 0;
  return map(constrain(altitude - zero, 0, 60), 0, 60, 60, 180);
}

void updateMotorSpeed(double x, double y, int baseSpeed)
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
    Serial.print("%");
  }
  Serial.println();

  for (int i = 0; i < 4; ++i)
  {
    vitesseMoteur[i] = constrain(baseSpeed * (pourcentage[i] / 100.0), 0, 180);
    Serial.print("  M");
    Serial.print(i);
    Serial.print(" : ");
    Serial.print(vitesseMoteur[i]);
  }
  Serial.println();
}

void updateAltitude()
{
  long new_altitude = ultrasonic.MeasureInCentimeters();
  delta_altitude = altitude - new_altitude;
  if (new_altitude >= 300)
    return;
  altitude = new_altitude;
}

void calibrateZero()
{
  updateAltitude();
  long alt = altitude;
  delay(1000);
  updateAltitude();
  if (abs(altitude - alt) > 5)
  {
    delay(1000);
    updateAltitude();
  }
  zero = altitude;
}

void setup()
{
  Serial.begin(19200);
  Wire.begin();
  sixDOF.init();

  for (int i = 0; i < 4; i++)
  {
    moteur[i].attach(i);
    moteur[i].write(0);
  }
  

  Serial.println("CHEIRODRONE");
  Serial.println("-----------------------------------");
  Serial.println();
  Serial.println("Calibrating for 3 seconds");
  delay(3000);
  calibrateZero();
  Serial.println("Calibration finished");
}

void loop()
{
  sixDOF.getRawValues(rawSixDof);
  angle[0] = _atan2(rawSixDof[0],rawSixDof[2]);
  angle[1] = _atan2(rawSixDof[1],rawSixDof[2]);

  updateAltitude();
  updateMotorSpeed(angle[0] / 10.0, angle[1] / 10.0, getMotorBaseSpeed());

  for (int i = 0; i < 4; i++)
  {
    moteur[i].write(vitesseMoteur[i]);
  }
  
  Serial.print("Altitude : ");
  Serial.print(altitude);
  Serial.print(" cm - Altitude C : ");
  Serial.print(altitude - zero);
  Serial.println(" cm");
  
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


