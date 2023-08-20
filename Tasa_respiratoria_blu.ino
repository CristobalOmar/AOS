#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

int c_ = 0;
void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32_TASA_RESPIRATORIA"); // Nombre del dispositivo Bluetooth
  pinMode(34, INPUT); // Configura el pin 34 como entrada analógica
}

void loop() {
  int sensorValue = analogRead(34); // Lee el valor analógico del pin 34
  float voltage = sensorValue * (3.3 / 4095.0); // Calcula el voltaje correspondiente
  Serial.print(c_);
  Serial.print(";Sensor value: ");
  Serial.print(sensorValue);
  Serial.print(", Voltage: ");
  Serial.println(voltage, 2);

  SerialBT.print(c_);
  SerialBT.print(";TR");
  SerialBT.println(sensorValue); // Envía el voltaje por Bluetooth
  c_ += 1;
  delay(50); // Espera 50 ms antes de repetir el bucle
}
