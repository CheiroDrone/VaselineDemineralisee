// Ce fichier fait partie du projet ChéiroDrone.
// Référez-vous au fichier LICENSE pour les informations sur la license.

#include <math.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include "cheirogant.h"

static FreeSixIMU sixDOF = FreeSixIMU();
int g_values[6];
int g_angle_x;
int g_angle_y;

void setup() {
  Serial.begin(9600);
  sixDOF.init();
}

void loop() {
  sixDOF.getRawValues(g_values);
  auto* values = reinterpret_cast<Values*>(g_values);
  g_angle_x = atan2(g_values[0], g_values[2]);
  g_angle_y = atan2(g_values[1], g_values[2]);
  Serial.print(" X: ");
  Serial.print(g_angle_x);
  Serial.print(" Y: ");
  Serial.print(g_angle_y);
  Serial.print(" acc.Z: ");
  // 230 correspond à la valeur de l'accélération z quand il n'y a pas de mouvement
  // (dépend de la pesanteur ; 230 à Paris)
  Serial.print(map(values->acc.z - 230, -512, 512, 0, 100));
  Serial.println();
}
