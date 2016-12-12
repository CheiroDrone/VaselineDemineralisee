#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

FreeSixIMU sixDOF = FreeSixIMU();
int g_values[6];
int16_t g_angles[2];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sixDOF.init();
}

void loop() {
  sixDOF.getRawValues(g_values);
  g_angles[0] = _atan2(g_values[0], g_values[2]);
  g_angles[1] = _atan2(g_values[1], g_values[2]);
  Serial.print(" X: ");
  Serial.print(g_angles[0] / 10.0);
  Serial.print(" Y: ");
  Serial.print(g_angles[1] / 10.0);
  Serial.println();
  delay(100);
}

int16_t _atan2(int32_t y, int32_t x)
{
  float z = (float)(y / x);
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
