#include "ChargerController.h"

ChargerController charger;

void setup() {
  charger.begin();
}

void loop() {
  charger.update();
}
