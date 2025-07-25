
#include <MT6701_I2C.h>

MT6701I2C SensorI2C(&Wire);

void setup() {
  Serial.begin(9600);

  SensorI2C.begin();
  SensorI2C.setClock();

  while (!SensorI2C.isConnected()) {
    Serial.println("Sensor not Connected!");
    delay(500);
  }
  Serial.println("Sensor Found!");
  Serial.println();

  delay(300);  // The delay is not important. so it's easier to see in the SM

  Serial.println("Reading Current Value...");
  word zpw;
  zpw = SensorI2C.getZPulseWidth();
  Serial.print("Zero Pulse Width (HEX): ");
  Serial.println(zpw, HEX);
  Serial.println();

  delay(300);  // The delay is not important. so it's easier to see in the SM

  Serial.print("Changing Value... ");
  SensorI2C.setZPulseWidth1LSB(); //YOU MODIFY THIS

  delay(300);  // The delay is not important. so it's easier to see in the SM

  Serial.println("Saving New Values...");
  Serial.println();
  // It's important to save the new values after the change.
  // Called once even after setting multiple values
  // else values return to default after power off
  SensorI2C.saveNewValues();

  delay(700);  // >600ms

  Serial.println("Reading New Value...");
  
  zpw = SensorI2C.getZPulseWidth();
  Serial.print("Zero Pulse Width (HEX): ");
  Serial.println(zpw, HEX);
  Serial.println();

  delay(300);  // The delay is not important. so it's easier to see in the SM

  Serial.println("Program Complete!");
}

void loop() {
  // nop
}
