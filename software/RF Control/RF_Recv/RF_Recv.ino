#include <Arduino.h>
#include <EEPROM.h>
#include <RFBeeSendRev.h>
#include <RFBeeCore.h>
#include <Wire.h>

#define FRAMESTART1               0x53  // data frame start1
#define FRAMESTART2               0x01  // data frame start2
#define FRAMEEND1                 0x2f  // data frame end1
#define FRAMEEND2                 0x45  // data frame end2

#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
#define Stepernu                  0x1c
#define I2CMotorDriverAdd1        0x0f  //Set the address of the I2CMotorDriver1
#define I2CMotorDriverAdd2        0x0e  //Set the address of the I2CMotorDriver2

#define JOYSTICK_PARA             512
#define MOVE_THRESHOLD            20
#define L1                        100
#define L2                        77
#define R                         30
#define DEBUG                     false

unsigned char rxData[20];
unsigned char len;
unsigned char srcAddress;
unsigned char destAddress;
char rssi;
unsigned char lqi;
int r_speed;
int x_speed;
int y_speed;
int w1,w2,w3,w4;
int _w1,_w2, _w3, _w4;
uint8_t I2CMotorDriverAdd;
uint8_t recvData[20];

void setup()
{
  Serial.begin(9600);
  RFBEE.init();
  w1 = w2 = w3 = w4 = 0;
  _w1 = _w2 = _w3 = _w4 = 0;
  Wire.begin();
}

void loop()
{
   if(RFBEE.isDta()){
       int result = receiveData(rxData, &len, &srcAddress, &destAddress, (unsigned char *)&rssi , &lqi);
       if((len == 10) && (rxData[0] == FRAMESTART1) && (rxData[1] == FRAMESTART2) && (rxData[8] == FRAMEEND1) && (rxData[9] == FRAMEEND2)){
          r_speed = (rxData[2]<<8|rxData[3]) - JOYSTICK_PARA;
          x_speed = (rxData[4]<<8|rxData[5]) - JOYSTICK_PARA;
          y_speed = (rxData[6]<<8|rxData[7]) - JOYSTICK_PARA;
#if DEBUG
          Serial.print("r = ");
          Serial.println(r_speed);
          Serial.print("x = ");
          Serial.println(x_speed);
          Serial.print("y = ");
          Serial.println(y_speed);
#endif
          updateSpeedPara();
          updateAngularSpeed();
          if((w1 != _w1) || (w2 != _w2) || (w3 != _w3) || (w4 != _w4)){
              _w1 = w1;
              _w2 = w2;
              _w3 = w3;
              _w4 = w4;
              updateMotoDriver();
          }
       }
   }
}

void updateSpeedPara()
{
    if((abs(x_speed)<MOVE_THRESHOLD) && (abs(y_speed)<MOVE_THRESHOLD)){
        x_speed = 0; 
        y_speed = 0;
    }
    if(abs(x_speed)/(abs(y_speed)+1) >= 2){
        y_speed = 0;
    }else if(abs(y_speed)/(abs(x_speed)+1) >= 2){
        x_speed = 0;
    }else{
        int tmp = abs(x_speed)/2 + abs(y_speed)/2;
        x_speed = x_speed<0?-tmp:tmp;
        y_speed = y_speed<0?-tmp:tmp;
    }
    if(abs(r_speed) < 50){
        r_speed = 0;
    }
    x_speed = x_speed*10;
    y_speed = y_speed*10;
    r_speed = r_speed/40;
#if DEBUG
    Serial.print("x_speed = ");
    Serial.println(x_speed);
    Serial.print("y_speed = ");
    Serial.println(y_speed);
    Serial.print("r_speed = ");
    Serial.println(r_speed);
#endif
}

void updateAngularSpeed()
{
    w1 = (x_speed + y_speed - r_speed*L1 - r_speed*L2)/R;
    w2 = (x_speed - y_speed + r_speed*L1 + r_speed*L2)/R;
    w3 = (x_speed - y_speed - r_speed*L1 - r_speed*L2)/R;
    w4 = (x_speed + y_speed + r_speed*L1 + r_speed*L2)/R;
#if DEBUG
    Serial.print("w1 = ");
    Serial.println(w1);
    Serial.print("w2 = ");
    Serial.println(w2);
    Serial.print("w3 = ");
    Serial.println(w3);
    Serial.print("w4 = ");
    Serial.println(w4);
#endif
}

void updateMotoDriver()
{
    I2CMotorDriverAdd = I2CMotorDriverAdd1;
    uint8_t dire = updateMotorDirection(w1,w2);
    MotorDriectionAndSpeedSet(dire, abs(w1), abs(w2));
#if DEBUG
    Serial.print("direction = ");
    Serial.println(dire);
    Serial.print("w1 = ");
    Serial.println(w1);
    Serial.print("w2 = ");
    Serial.println(w2);
#endif
    I2CMotorDriverAdd = I2CMotorDriverAdd2;
    dire = updateMotorDirection(w3,w4);
    MotorDriectionAndSpeedSet(dire, abs(w3), abs(w4));    
#if DEBUG
    Serial.print("direction = ");
    Serial.println(dire);
    Serial.print("w3 = ");
    Serial.println(w3);
    Serial.print("w4 = ");
    Serial.println(w4);
#endif
}

uint8_t updateMotorDirection(int wx, int wy)
{
    if((wx <=0) && (wy <= 0)){
        return 0b0110;
    }else if((wx >= 0) && (wy >= 0)){
        return 0b1001;
    }else if((wx < 0) && (wy > 0)){
        return 0b1010;
    }else{
        return 0b0101;
    }
}

void MotorSpeedSetAB(unsigned char MotorSpeedA , unsigned char MotorSpeedB)  
{
  MotorSpeedA=map(MotorSpeedA,0,100,0,255);
  MotorSpeedB=map(MotorSpeedB,0,100,0,255);
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);                 // set pwm header 
  Wire.write(MotorSpeedA);                   // send pwma 
  Wire.write(MotorSpeedB);                   // send pwmb    
  Wire.endTransmission();                    // stop transmitting
}

void MotorPWMFrequenceSet(unsigned char Frequence)  
{    
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(PWMFrequenceSet);               // set frequence header
  Wire.write(Frequence);                     //  send frequence 
  Wire.write(Nothing);                       //  need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();                    // stop transmitting
}

void MotorDirectionSet(unsigned char Direction)        //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
{     
  Wire.beginTransmission(I2CMotorDriverAdd);           // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);                            // Direction control header
  Wire.write(Direction);                               // send direction control information
  Wire.write(Nothing);                                 // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();                              // stop transmitting 
}

void MotorDriectionAndSpeedSet(unsigned char Direction,unsigned char MotorSpeedA,unsigned char MotorSpeedB)//you can adjust the driection and speed together
{
  MotorDirectionSet(Direction);
  MotorSpeedSetAB(MotorSpeedA,MotorSpeedB);  
}



