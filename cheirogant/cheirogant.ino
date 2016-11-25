#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

#include "cheirogant.h"
#include "math.h"

static FreeSixIMU sixDOF = FreeSixIMU();
Values g_values;
int16_t g_angle_x;
int16_t g_angle_y;
int g_last_acc_z;
int g_motor_power;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  sixDOF.init();
}

void loop() {
  sixDOF.getRawValues(reinterpret_cast<int*>(&g_values));
  g_angle_x = _atan2(g_values.acc.x, g_values.acc.z);
  g_angle_y = _atan2(g_values.acc.y, g_values.acc.z);
  Serial.print(" X: ");
  Serial.print(g_angle_x / 10.0);
  Serial.print(" Y: ");
  Serial.print(g_angle_y / 10.0);
  Serial.print(" acc.Z: ");
  // 230 correspond à la valeur de l'accélération z quand il n'y a pas de mouvement
  // (dépend de la pesanteur ; 230 à Paris)
  const int z = map(g_values.acc.z - 230, -512, 512, 0, 100);
  Serial.print(z);
  Serial.print(" difference : ");
  Serial.print(z - g_last_acc_z);
  Serial.println();
  delay(100);
  g_last_acc_z = z;
}
