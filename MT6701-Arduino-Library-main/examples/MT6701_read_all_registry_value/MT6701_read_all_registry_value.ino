#include "MT6701_I2C.h"

// Crea un'istanza del sensore MT6701
MT6701I2C sensor(&Wire);

void setup() {
  // Inizia la comunicazione seriale
  Serial.begin(9600);

  // Inizia la comunicazione I2C
  sensor.begin();

  /*
  sensor.MT_WriteOneRegister(0x25, 0b10000000);
  sensor.MT_WriteOneRegister(0x26, 0b00000000);
  sensor.MT_WriteOneRegister(0x27, 0b11011000);
  sensor.MT_WriteOneRegister(0x28, 0b00000100);
  sensor.MT_WriteOneRegister(0x29, 0b00000100);
  sensor.MT_WriteOneRegister(0x2A, 0b00000000);
  sensor.MT_WriteOneRegister(0x2B, 0b00000000);
  sensor.MT_WriteOneRegister(0x2C, 0b00000000);
  sensor.MT_WriteOneRegister(0x2D, 0b00000000);
  sensor.MT_WriteOneRegister(0x2E, 0b00000000);
  sensor.MT_WriteOneRegister(0x2F, 0b00000000);
  sensor.MT_WriteOneRegister(0x30, 0b00001111);
  sensor.MT_WriteOneRegister(0x31, 0b11100111);
  sensor.MT_WriteOneRegister(0x32, 0b00000000);
  sensor.MT_WriteOneRegister(0x33, 0b00000000);
  sensor.MT_WriteOneRegister(0x34, 0b00000111);
  sensor.MT_WriteOneRegister(0x35, 0b10101001);
  sensor.MT_WriteOneRegister(0x36, 0b00111011);
  sensor.MT_WriteOneRegister(0x37, 0b11100111);
  sensor.MT_WriteOneRegister(0x38, 0b00100011);
  sensor.MT_WriteOneRegister(0x39, 0b11100000);
  sensor.MT_WriteOneRegister(0x3A, 0b00000111);
  sensor.MT_WriteOneRegister(0x3B, 0b10101001);
  sensor.MT_WriteOneRegister(0x3C, 0b00111011);
  sensor.MT_WriteOneRegister(0x3D, 0b11100111);
  sensor.MT_WriteOneRegister(0x3E, 0b00000000);
  sensor.MT_WriteOneRegister(0x3F, 0b00000000);
  sensor.MT_WriteOneRegister(0x40, 0b00000000);
*/

 // sensor.saveNewValues();
 // delay(700);  // >600мс

  // Stampa una linea vuota per separare l'output
  Serial.println();

  // Stampa lo stato di tutti i registri
  Serial.println("Valori di tutti i registri: (da 0x25 a 0x40)");
  Serial.println("|bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|");
  sensor.printAllRegisterValue();
}

void loop() {
  // Non c'è nulla da fare nel loop principale in questo esempio
}
