#include "MT6701_I2C.h"

// Crea un'istanza del sensore MT6701
MT6701I2C sensor(&Wire);

void setup() {
  // Inizia la comunicazione seriale
  Serial.begin(9600);

  // Inizia la comunicazione I2C
  sensor.begin();

  Serial.println("Write Resolution ABZ = 1000");
  //word new_res_abz = 732;
  if(sensor.setOutputResolutionABZVerify(1000)) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }

  sensor.saveNewValues();
  delay(700); // >600мс

  // Stampa una linea vuota per separare l'output
  Serial.println();

  // Stampa lo stato di tutti i registri
  Serial.println("Stato di tutti i registri:");
  sensor.printAllRegisterStates();
}

void loop() {
  // Non c'è nulla da fare nel loop principale in questo esempio
}

