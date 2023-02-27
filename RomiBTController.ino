
#include <Wire.h>
#include <PS4Controller.h>
#include "romi_registers.h"
#include "secrets.h"

#define I2C_DEV_ADDR 20

struct Data
{
  bool yellow, green, red;
  bool buttonA, buttonB, buttonC;
  int16_t leftMotor, rightMotor;
  
  uint16_t batteryMillivolts;
  uint16_t analog[6];

  bool playNotes;
  char notes[14];

  int16_t leftEncoder, rightEncoder;
  uint16_t liftServo, tiltServo, gripperServo;
};

Data slave;
uint16_t gripValue = 600; //550 = open, 2350 = closed
uint16_t tiltValue = 1540; //1380 = most down, 1770 = most up
uint16_t liftValue = 1790; //800 = lowest, 1790 = highest
int16_t leftMotor = 0;
int16_t rightMotor = 0;
int8_t mode = 1;

void romi_i2c_read(uint8_t address, uint8_t *data, uint8_t size){

  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(address);
  uint8_t error = Wire.endTransmission(true);
  delay(1);
  uint8_t bytesReceived = Wire.requestFrom((uint8_t)I2C_DEV_ADDR, size);
  if((bool)bytesReceived){ //If received more than zero bytes
    Wire.readBytes(data, bytesReceived);
    //Serial.write(data, size);
  }
}

void romi_i2c_write(uint8_t address, uint8_t *data, uint8_t size){

  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(address);
  Wire.write(data, size);
  uint8_t error = Wire.endTransmission(true);

}

void romi_setMotors(int16_t left, int16_t right){
  uint8_t buffer[4];
  memcpy(&buffer[0], &left, sizeof(left));
  memcpy(&buffer[2], &right, sizeof(right));
  romi_i2c_write(MOTOR_ADDR, buffer,4);
}

void romi_setServos(uint16_t lift, uint16_t tilt,uint16_t gripper){
  uint8_t buffer[6];
  memcpy(&buffer[0], &lift, sizeof(lift));
  memcpy(&buffer[2], &tilt, sizeof(tilt));
  memcpy(&buffer[4], &gripper, sizeof(gripper));
  romi_i2c_write(SERVO_ADDR, buffer,6);
}

void romi_getBattery(void){
  uint8_t buffer[2];
  romi_i2c_read(BATTERY_ADDR, buffer, 2);
  slave.batteryMillivolts = buffer[1]<<8 | buffer[0];
}

void romi_getAnalogs(void){
  uint8_t buffer[12];
  romi_i2c_read(ANALOG_ADDR, buffer, 12);
  for(uint8_t i=0; i<6; i++){
    slave.analog[i] = buffer[i*2+1]<<8 | buffer[i*2];    
  }
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  PS4.begin("58:24:29:52:c1:c1");
  Serial.println("Ready.");

}

void loop()
{
  leftMotor = 0;
  rightMotor = 0;
  if (PS4.isConnected()) {
    Serial.println("Connected");
    /*if (abs(PS4.LStickY()) > 5){
      leftMotor = map(PS4.LStickY(), -128, 128, -300, 300);
    }
    if (abs(PS4.RStickY()) > 5){
      rightMotor = map(PS4.RStickY(), -128, 128, -300, 300);
    }*/
    if (PS4.L1() & PS4.R1()){
      mode = 2;
    }else{
      mode = 1;
    }
    if (PS4.Up()){
      leftMotor = 80;
      rightMotor = 80;
    }
    if (PS4.UpRight()){
      leftMotor = 60;
      rightMotor = 0;
    }
    if (PS4.Right()){
      leftMotor = 40;
      rightMotor = -40;
    }
    if (PS4.DownRight()){
      leftMotor = 0;
      rightMotor = -60;
    }
    if (PS4.Down()){
      leftMotor = -80;
      rightMotor = -80;
    }
    if (PS4.DownLeft()){
      leftMotor = -60;
      rightMotor = 0;
    }
    if (PS4.Left()){
      leftMotor = -40;
      rightMotor = 40;
    }
    if (PS4.UpLeft()){
      leftMotor = 0;
      rightMotor = 60;
    }
    if (PS4.Triangle()) {
      liftValue -= 17;
      tiltValue -= 4;
    }
    if (PS4.Cross()) {
      liftValue += 17;
      tiltValue += 4;
    }
    if (PS4.Square()) {
      gripValue += 30;
    }
    if (PS4.Circle()) {
      gripValue -= 30;
    }
    /*if (PS4.L1()){
      tiltValue -= 10;
    }
    if (PS4.R1()){
      tiltValue += 10;
    }*/
    liftValue = constrain(liftValue, 1110, 1790);
    tiltValue = constrain(tiltValue, 1380, 1540);
    gripValue = constrain(gripValue, 550, 1800);
    romi_setServos(liftValue,tiltValue,gripValue);
    romi_setMotors(leftMotor*mode,rightMotor*mode);  
    if (slave.batteryMillivolts < 7000){
      PS4.setLed(255, 0, 0);
    }else{
      PS4.setLed(0, 255, 0);
    }  
    PS4.sendToController();
  }
  
  romi_getBattery();
  //Serial.print("Batt V: ");
  //Serial.println(slave.batteryMillivolts);
  Serial.printf("H: %d , T: %d , G: %d\n", liftValue, tiltValue, gripValue);
  /*romi_getAnalogs();
  Serial.print("Analog: ");
  Serial.print((float)slave.analog[0]/1024*5);
  Serial.print(" , ");
  Serial.print((float)slave.analog[3]/1024*5);
  Serial.print(" , ");
  Serial.println((float)slave.analog[4]/1024*5);
  */
  delay(100);
}
