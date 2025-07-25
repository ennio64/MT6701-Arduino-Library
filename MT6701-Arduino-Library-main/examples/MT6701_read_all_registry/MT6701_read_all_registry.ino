#include "MT6701_I2C.h"

// Crea un'istanza del sensore MT6701
MT6701I2C sensor(&Wire);

void setup() {
  // Inizia la comunicazione seriale
  Serial.begin(9600);

  // Inizia la comunicazione I2C
  sensor.begin();

  // Stampa una linea vuota per separare l'output
  Serial.println();

  // Stampa lo stato di tutti i registri
  Serial.println("Stato di tutti i registri:");
  sensor.printAllRegisterStates();
}

void loop() {
  // Non c'è nulla da fare nel loop principale in questo esempio
}
