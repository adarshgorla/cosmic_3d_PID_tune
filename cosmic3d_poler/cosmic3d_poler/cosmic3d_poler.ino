// cosmic3d_poler.ino
// Modular ESP32-S3 Cosmic Polar 400 Main Entry Point

#include "PrinterStateMachine.h"

void setup() {
  cosmicPolarSetup();
}

void loop() {
  cosmicPolarLoop();
}
