#include "ChargerController.h"
#include "Command.h"

ChargerController charger;
Command commandLink(charger);

void setup() {
  charger.begin();
  commandLink.begin();
}

void loop() {
  charger.update();
  commandLink.update();
}
