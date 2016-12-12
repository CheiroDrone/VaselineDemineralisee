#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
int16_t angle[2]; // pitch & roll
void setup() 
{ 
  Serial.begin(9600);
  Wire.begin();
  sixDOF.init();
}

static constexpr int OFFSET = 20;
float get_rotation_speed_delta()
{
  static float last_position = 0;
  const int gyro_z = rawSixDof[5] + OFFSET;
  const float value = map(constrain(gyro_z, -2000 + OFFSET, 2000 + OFFSET), 2000 + OFFSET, -2000 + OFFSET, -100, 100) / 100.0;
  const float new_value = constrain(last_position + value * 0.06, -1, 1);
  if (abs(value) > 0.05)
    last_position = new_value;
  if (abs(rawSixDof[2]) > 300)
    last_position = 0;
  return last_position;
}

int get_rotation_speed()
{
  static int last_speed = 0;
  const float delta = get_rotation_speed_delta();
  Serial.print(" delta: ");
  Serial.print(delta);
  if (abs(delta) > 0.05)
    last_speed = constrain(last_speed + get_rotation_speed_delta() * 180 * 0.2, 0, 180);
  Serial.print(" motor speed (0-180): ");
  Serial.print(last_speed);
  return last_speed;
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
  Serial.print(" acc Z: ");
  Serial.print(rawSixDof[2]);
  Serial.print(" gyroX: ");
  Serial.print(rawSixDof[3]);
  Serial.print(" gyroY: ");
  Serial.print(rawSixDof[4]);
  Serial.print(" gyroZ: ");
  Serial.print(rawSixDof[5]);
  Serial.println();
  get_rotation_speed();
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
