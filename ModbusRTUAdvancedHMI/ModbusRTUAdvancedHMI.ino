#include <Modbus.h>
#include <ModbusSerial.h>

#include <FastPID.h>

ModbusSerial mb;

//fastpid globals
float Kp=1, Ki=15, Kd=0, Hz=10;
int output_bits = 8;
bool output_signed = false;

FastPID FELSO_PID(Kp, Ki, Kd, Hz, output_bits, output_signed);
FastPID FORRALO_PID(Kp, Ki, Kd, Hz, output_bits, output_signed);

//Input reg. address:
const int MASLO_REG = 0;
const int FORRALO_REG =1;
const int ALSO_REG=2;
const int FELSO_REG=3;
const int PWR_IN =4;
const int PWR_OUT = 5;

//holding reg. address:
const int FORRALOSP_REG = 6;
const int FELSOSP_REG=7;

//coil address:
const int FORRALO_EN = 0;
const int FELSO_EN = 1;
const int OUT_EN = 2;

void setup() {
  
    mb.config(&Serial, 38400, SERIAL_8N1);
    mb.setSlaveId(1);  
    
    mb.addHreg(MASLO_REG);
    mb.addHreg(FORRALO_REG);
    mb.addHreg(ALSO_REG);
    mb.addHreg(FELSO_REG);
    mb.addHreg(PWR_IN);
    mb.addHreg(PWR_OUT);
    mb.addHreg(FORRALOSP_REG);
    mb.addHreg(FELSOSP_REG);

    mb.addCoil(FORRALO_EN);
    mb.addCoil(FELSO_EN);
    mb.addCoil(OUT_EN);
    
    pinMode(13,OUTPUT);
    digitalWrite(13,LOW);
    Serial1.begin(9600);

}

uint8_t pwr = 0;
unsigned long tmr, pwm;
uint16_t T0,T1,T2,T3;
uint8_t b1;
uint8_t b2;

void loop() {
  
    mb.task();
  //pwm generáló:  
    if(mb.Coil(OUT_EN)){
      
    if(pwm + pwr*4 >= millis()){
      digitalWrite(13,HIGH);
      }
    else if ((pwm + 1024) >= millis()){
      digitalWrite(13,LOW);
      }
    else {
      pwm = millis();}
    }
    else{digitalWrite(13,LOW);}
  
  //1024 millis enként futó:
    if(tmr + 1000 <= millis()){

        
        tmr = millis();
        Serial1.write('0');
        b1  = Serial1.read();
        b2 =  Serial1.read();
        T0 = (((b1 << 0) & 0xFF) + ((b2 << 8) & 0xFFFF));

        Serial1.write('1');
        b1  = Serial1.read();
        b2 =  Serial1.read();
        T1 = (((b1 << 0) & 0xFF) + ((b2 << 8) & 0xFFFF));
        
        Serial1.write('2');
        b1  = Serial1.read();
        b2 =  Serial1.read();
        T2 = (((b1 << 0) & 0xFF) + ((b2 << 8) & 0xFFFF));
        
        Serial1.write('3');
        b1  = Serial1.read();
        b2 =  Serial1.read();
        T3 = (((b1 << 0) & 0xFF) + ((b2 << 8) & 0xFFFF));

        Serial1.write('4');
        
    }
        //sensors.requestTemperatures();
        mb.Hreg(MASLO_REG, T0);
        mb.Hreg(FORRALO_REG, T1);
        mb.Hreg(ALSO_REG,T2);
        mb.Hreg(FELSO_REG,T3);
        
    if (mb.Coil(FORRALO_EN)){
      pwr = FORRALO_PID.step(mb.Hreg(FORRALOSP_REG), mb.Hreg(FORRALO_REG));
      mb.Hreg(PWR_OUT, pwr);
      }
    else if(mb.Coil(FELSO_EN)){
      pwr = FELSO_PID.step(mb.Hreg(FELSOSP_REG), mb.Hreg(FELSO_REG));
      mb.Hreg(PWR_OUT, pwr);
      }
    else {
      pwr = mb.Hreg(PWR_IN);
      mb.Hreg(PWR_OUT, pwr);
      }
}
