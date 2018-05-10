#include <OneWire.h> 
#include <DallasTemperature.h>


OneWire oneWire(2); 
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(9600);

  sensors.begin();

}

uint16_t T0,T1,T2,T3;
unsigned char command;
bool enable = false;

void loop() {

  if(enable == true){
        sensors.requestTemperatures();
        T0 = (sensors.getTempCByIndex(0)*10);
        T1 = (sensors.getTempCByIndex(1)*10);
        T2 = (sensors.getTempCByIndex(2)*10);
        T3 = (sensors.getTempCByIndex(3)*10);
        enable = false;
        }

if (Serial.available() > 0) {
        command = Serial.read();
        
        switch (command){
          case '0':
            Serial.write(T0 & 0xFF); 
            Serial.write((T0 >> 8) & 0xFF);
            break;
          case '1':
            Serial.write(T1 & 0xFF); 
            Serial.write((T1 >> 8) & 0xFF);
            break;
          case '2':
            Serial.write(T2 & 0xFF); 
            Serial.write((T2 >> 8) & 0xFF);
            break;
          case '3':
            Serial.write(T3 & 0xFF); 
            Serial.write((T0 >> 8) & 0xFF);
            break;
          case '4':
            enable = true;
            break;
            }
         
        }
}
