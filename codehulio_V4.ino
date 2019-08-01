#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"

Servo FL_prop;
Servo FR_prop;
Servo BL_prop;
Servo BR_prop;
///////////////////kirikanan
Servo motor_kiri;
Servo motor_kanan;
int kirikanan = 1;
int atasbawah = 2;
#define l_relay 10
#define r_relay 11
int map_kirikanan;
int map_atasbawah;
int LmotorValue;
int RmotorValue;
int Value = 1500;
///////////////end-kirikanan
int FL_relay = 9;
int FR_relay = 8;
int BL_relay = 13;
int BR_relay = 12;
//int kiri_relay = 11;
//int kanan_relay = 10;
int tambahan = 60;

///////////////////CALIBRATION
////Change this 3 variables if you want to fine tune the skecth to your needs.
//int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
//int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
//int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
//
//// default I2C address is 0x68
//// specific I2C addresses may be passed as a parameter here
//// AD0 low = 0x68 (default for InvenSense evaluation board)
//// AD0 high = 0x69
////MPU6050 accelgyro;
//MPU6050 accelgyro(0x68); // <-- use for AD0 high
//
//int16_t ax, ay, az,gx, gy, gz;
//
//int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
//int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

//////////////////// MPU-6050//////////////////////////////
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float alpha;
float rad_to_deg = 180/3.141592654;

/////////////////PID CONSTANTS/////////////////
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_pX=0, pid_pY=0;
float pid_iX=0, pid_iY=0;
float pid_dX=0, pid_dY=0;
/// X axis
double kpX=12;//3.55
double kiX=0.005;//0.003
double kdX=3;//2.05
// Y axis
double kpY=12 ;//3.55
double kiY=0.005;//0.003
double kdY=3;//2.05

//////////////////MOTOR WRITE/////////////
float zero = 0; //This is the angle in which we whant the
                         //balance to stay steady
float errorX, errorY;
float preverrorX,preverrorY;
float PIDX, PIDY;
int MFR, MFL, MBR, MBL;
int X;
float throttle =1000;
float PIDBL, PIDBR, PIDFL, PIDFR;

void setup() {

//  TWBR = 24;
//     //////////////////// CALIBRATION///////////////////
  Serial.begin(9600);
  Wire.begin();
//  accelgyro.initialize();
//  // start message
//  Serial.println("\nMPU6050 Calibration Sketch");
//  //delay(2000);
//  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
//  //delay(3000);
//  // verify connection
//  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//  delay(1000);
//  // reset offsets
//  accelgyro.setXAccelOffset(713);
//  accelgyro.setYAccelOffset(98);
//  accelgyro.setZAccelOffset(17098);
//  accelgyro.setXGyroOffset(0);
//  accelgyro.setYGyroOffset(0);
//  accelgyro.setZGyroOffset(0);

  
  ////////// MPU-6050/////////////
                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
//  // Call this function if you need to get the IMU error values for your module
//  if (state==0){
//    Serial.println("\nReading sensors for first time...");
//    meansensors();
//    state++;
//    delay(1000);
//  }
//
//  if (state==1) {
//    Serial.println("\nCalculating offsets...");
//    calibration();
//    state++;
//    delay(1000);
//  }
//
//  if (state==2) {
//    meansensors();
//    Serial.println("\nFINISHED!");
////    Serial.print("\nSensor readings with offsets:\t");
////    Serial.print(mean_ax); 
////    Serial.print("\t");
//    Serial.print(mean_ay); 
//    Serial.print("\t");
//    Serial.print(mean_az); 
//    Serial.print("\t");
//    Serial.print(mean_gx); 
//    Serial.print("\t");
//    Serial.print(mean_gy); 
//    Serial.print("\t");
//    Serial.println(mean_gz);
//    Serial.print("Your offsets:\t");
//    Serial.print(ax_offset); 
//    Serial.print("\t");
//    Serial.print(ay_offset); 
//    Serial.print("\t");
//    Serial.print(az_offset); 
//    Serial.print("\t");
//    Serial.print(gx_offset); 
//    Serial.print("\t");
//    Serial.print(gy_offset); 
//    Serial.print("\t");
//    Serial.println(gz_offset); 
//    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
//    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
//    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
//  }
  calculate_IMU_error();
  delay(20);

  //////////////////////////////kirikanan
  pinMode(l_relay, OUTPUT);
  pinMode(r_relay, OUTPUT);
  motor_kiri.attach(5);  // attaches the servo on pin 9 to the servo object
  motor_kanan.attach(4);
  //////////////////////////end-kirikanan
  pinMode(FL_relay, OUTPUT);
  pinMode(FR_relay, OUTPUT);
  pinMode(BL_relay, OUTPUT);
  pinMode(BR_relay, OUTPUT);
  Wire.begin(); //begin the wire comunication
  FL_prop.attach(2); //attatch the right motor to pin 3 MOTOR 2
  FR_prop.attach(3); //attatch the left motor to pin 2 1
  BL_prop.attach(6); //attatch the right motor to pin 7 6
  BR_prop.attach(7); // PIN 6 MOTOR 5
////  
//  FL_prop.writeMicroseconds(1000);
//  FR_prop.writeMicroseconds(1000);
//  BL_prop.writeMicroseconds(1000);
//  BR_prop.writeMicroseconds(1000);
//
//  FL_prop.writeMicroseconds(1000);
//  FL_prop.writeMicroseconds(1500);
//  delay(1000);
//  FL_prop.writeMicroseconds(1000);
//  
//  FR_prop.writeMicroseconds(1000);
//  FR_prop.writeMicroseconds(1500);
//  delay(1000);
//  FR_prop.writeMicroseconds(1000);
//  
//  BL_prop.writeMicroseconds(1000);
//  BL_prop.writeMicroseconds(1500);
//  delay(1000);
//  BL_prop.writeMicroseconds(1000);
//  
//  BR_prop.writeMicroseconds(1000);
//  BR_prop.writeMicroseconds(1500);
//  delay(1000);
//  BR_prop.writeMicroseconds(1000);

  delay(1000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 
}//end of setup void

void loop() {
  X = 2*analogRead(A0);
  throttle = X-955;
  
  /////////////////////////////////kirikanan
  map_kirikanan = analogRead(kirikanan);
  map_atasbawah = analogRead(atasbawah);
  map_kirikanan = map(map_kirikanan, 0, 1023, -1000, 1000);
  map_atasbawah = map(map_atasbawah, 0, 1023, -1000, 1000);
  //-1000 -333 333 1000
  //4 pojok
  //kiri atas
  if(-1000 < map_kirikanan && map_kirikanan < -333 && 333 < map_atasbawah && map_atasbawah < 1000){
    Serial.print("kiriAtas ");
    LmotorValue = 1000;
    RmotorValue = Value;
    digitalWrite(l_relay, LOW); //LOW berarti maju
    digitalWrite(r_relay, LOW);
  }
  //kanan atas
  else if(333 < map_kirikanan && map_kirikanan < 1000 && 333 < map_atasbawah && map_atasbawah < 1000){
    Serial.print("kananAtas ");
    LmotorValue = Value;
    RmotorValue = 1000;
    digitalWrite(l_relay, LOW);
    digitalWrite(r_relay, LOW);
  }
  //kiri bawah
  else if(-1000 < map_kirikanan && map_kirikanan < -333 && -1000 < map_atasbawah && map_atasbawah < -333){
    Serial.print("kiriBawah ");
    LmotorValue = 1000;
    RmotorValue = Value;
    digitalWrite(l_relay, HIGH);
    digitalWrite(r_relay, LOW);
  }
  //kanan bawah
  else if(333 < map_kirikanan && map_kirikanan < 1000 && -1000 < map_atasbawah && map_atasbawah < -333){
    Serial.print("kananBawah ");
    LmotorValue = Value;
    RmotorValue = 1000;
    digitalWrite(l_relay, LOW);
    digitalWrite(r_relay, HIGH);
  }
  //kiri kiri
  else if(-1000 < map_kirikanan && map_kirikanan < -333 && -333 < map_atasbawah && map_atasbawah < 333){
    Serial.print("kiriKiri "); 
    LmotorValue = Value;
    RmotorValue = Value;
    digitalWrite(l_relay, HIGH);
    digitalWrite(r_relay, LOW);
  }
  //kanan kanan
  else if(333 < map_kirikanan && map_kirikanan < 1000 && -333 < map_atasbawah && map_atasbawah < 333){
    Serial.print("kananKanan "); 
    LmotorValue = Value;
    RmotorValue = Value;
    digitalWrite(l_relay, LOW);
    digitalWrite(r_relay, HIGH);
  }
  //atas atas
  else if(-333 < map_kirikanan && map_kirikanan < 333 && 333 < map_atasbawah && map_atasbawah < 1000){
    Serial.print("atasAtas "); 
    LmotorValue = 1000 -200 + map_atasbawah*1.15;
    RmotorValue = 1000 -200 + map_atasbawah*0.85;
    if (LmotorValue > 2000){
      LmotorValue = 2000;
    }
    if (RmotorValue > 2000){
      RmotorValue = 2000;
    }
    digitalWrite(l_relay, LOW);
    digitalWrite(r_relay, LOW);
  }
  //bawah bawah
  else if(-333 < map_kirikanan && map_kirikanan < 333 && -1000 < map_atasbawah && map_atasbawah < -333){
    Serial.print("bawahBawah "); 
    LmotorValue = abs(-1000 +200 + map_atasbawah*1.15);
    RmotorValue = abs(-1000 +200 + map_atasbawah*0.85);
    if (LmotorValue > 2000){
      LmotorValue = 2000;
    }
    if (RmotorValue > 2000){
      RmotorValue = 2000;
    }
    digitalWrite(l_relay, LOW);
    digitalWrite(r_relay, LOW);
    digitalWrite(l_relay, HIGH);
    digitalWrite(r_relay, HIGH);
  }
  else{
    LmotorValue = 1000;
    RmotorValue = 1000;
    digitalWrite(l_relay, LOW);
    digitalWrite(r_relay, LOW);
  }
//  Serial.print(" ");
//  Serial.print(map_kirikanan);
//  Serial.print(" ");
//  Serial.print(map_atasbawah);
//  Serial.print(" ");
//  Serial.print(LmotorValue);
//  Serial.print(" ");
//  Serial.print(RmotorValue);
//  Serial.println(" ");
  motor_kiri.writeMicroseconds(LmotorValue);
  motor_kanan.writeMicroseconds(RmotorValue);
  /////////////////////////////end-kirikanan
 
//  int RL = analogRead(A2);
//  int FB = analogRead(A1);

/////////////////////////////I M U/////////////////////////////////////
// put your main code here, to run repeatedly:
 // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY + GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ + GyroErrorZ; // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  alpha = 0.6321/(0.6321+elapsedTime);
  
  // Complementary filter - combine acceleromter and gyro angle values
  roll = ((1-alpha) *(roll+ gyroAngleX * elapsedTime) + alpha * accAngleX);
  pitch = ((1-alpha) *(pitch + gyroAngleY * elapsedTime)  + alpha * accAngleY);
  
  errorX = roll-zero;
  errorY = pitch-zero;
  preverrorX = errorX;
  preverrorY = errorY;
  error = errorX + errorY;
  
//////////////////////////PID CALC///////////////////////////

pid_pX = kpX*errorX*1.8;
pid_dX = kdX*((errorX-preverrorX)/elapsedTime)*2;
PIDX = pid_pX + pid_dX;

pid_pY = kpY*errorY*1.8;
pid_dY = kdY*((errorY-preverrorY)/elapsedTime)*2;
PIDY = pid_pY + pid_dY;

////////////////////////WRITE TO MOTOR///////////////////

if(errorX >=5 || errorX <=-5 || errorY >=5 || errorY <=-5)
{
 PIDBR=(+PIDX-PIDY);
 PIDFR=(+PIDX+PIDY);
 PIDBL=(-PIDX-PIDY);
 PIDFL=(-PIDX+PIDY);


if(roll<0){
  MBL = (PIDBL+throttle);
  MFL = (PIDFL+throttle);
  MBR = (PIDBR+throttle);
  MFR = (PIDFR+throttle);
}else{
  MBL = -(PIDBL+throttle);
  MFL = -(PIDFL+throttle);
  MBR = -(PIDBR+throttle);
  MFR = -(PIDFR+throttle);
}
if(pitch<0){
  MBL = (PIDBL+throttle);
  MFL = -(PIDFL+throttle);
  MBR = (PIDBR+throttle);
  MFR = -(PIDFR+throttle);
}else{
  MBL = -(PIDBL+throttle);
  MFL = (PIDFL+throttle);
  MBR = -(PIDBR+throttle);
  MFR = (PIDFR+throttle);
}}else{
 MBR=0+throttle;
 MFR=0+throttle;
 MBL=0+throttle;
 MFL=0+throttle;
}
////////////// LIMITER MOTOR///////////////////
if(MBR>2000){
  MBR = 2000;
}
if(MFR>2000){
  MFR = 2000;
}
if(MBL>2000){
  MBL = 2000;
}
if(MFL>2000){
  MFL = 2000;
}
if(MBR<-2000){
  MBR = 2000;
}
if(MBL<-2000){
  MBL = 2000;
}
if(MFR<-2000){
  MFR = 2000;
}
if(MFL<-2000){
  MFL = 2000;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
if(MFL<0){ 
  digitalWrite(FL_relay, HIGH);
}else{
 digitalWrite(FL_relay, LOW); 
}
if(MFR<0){
  digitalWrite(FR_relay, HIGH);
}else{
  digitalWrite(FR_relay, LOW);
}
if(MBL<0){
  digitalWrite(BL_relay, HIGH);
}else{
  digitalWrite(BL_relay, LOW);
}
if(MBR<0){
  digitalWrite(BR_relay, HIGH);
}else{
  digitalWrite(BR_relay, LOW);
}
MFL=abs(MFL)+1000;
MFR=abs(MFR)+1000;
MBL=abs(MBL)+1000;
MBR=abs(MBR)+1000;
FL_prop.writeMicroseconds(MFL);
FR_prop.writeMicroseconds(MFR);
BL_prop.writeMicroseconds(MBL);
BR_prop.writeMicroseconds(MBR);
//Serial.print(throttle);
//Serial.print(" throttle ");
Serial.print(MFL);
Serial.print(" FL ");
Serial.print(MFR);
Serial.print(" FR ");
Serial.print(MBL);
Serial.print(" BL ");
Serial.print(MBR);
Serial.print(" BR ||");
Serial.print(RmotorValue);
Serial.println(LmotorValue);
////Serial.print(state_FL);
//Serial.print(state_FR);
//Serial.print(state_BL);
//Serial.println(state_BR);
//Serial.print(PIDBR);
//Serial.print("//");
//Serial.print(PIDFR);
//Serial.print("//");
//Serial.print(PIDBL);
//Serial.print("//");
//Serial.println(PIDFL);

}//end of loop void

//void meansensors(){
//  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
//
//  while (i<(buffersize+101)){
//    // read raw accel/gyro measurements from device
//    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    
//    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
//      buff_ax=buff_ax+ax;
//      buff_ay=buff_ay+ay;
//      buff_az=buff_az+az;
//      buff_gx=buff_gx+gx;
//      buff_gy=buff_gy+gy;
//      buff_gz=buff_gz+gz;
//    }
//    if (i==(buffersize+100)){
//      mean_ax=buff_ax/buffersize;
//      mean_ay=buff_ay/buffersize;
//      mean_az=buff_az/buffersize;
//      mean_gx=buff_gx/buffersize;
//      mean_gy=buff_gy/buffersize;
//      mean_gz=buff_gz/buffersize;
//      Serial.print("buff_ax=");
//      Serial.print(buff_ax);
//      Serial.print(" buff_ay=");
//      Serial.print(buff_ay);
//      Serial.print(" buff_az=");
//      Serial.println(buff_az);
//    }
//    i++;
//    delay(2); //Needed so we don't get repeated measures
//  }
//}
//
//void calibration(){
//  ax_offset=-mean_ax/8;
//  ay_offset=-mean_ay/8;
//  az_offset=(16384-mean_az)/8;
//
//  gx_offset=-mean_gx/4;
//  gy_offset=-mean_gy/4;
//  gz_offset=-mean_gz/4;
//  while (1){
//    int ready=0;
//    accelgyro.setXAccelOffset(ax_offset);
//    accelgyro.setYAccelOffset(ay_offset);
//    accelgyro.setZAccelOffset(az_offset);
//
//    accelgyro.setXGyroOffset(gx_offset);
//    accelgyro.setYGyroOffset(gy_offset);
//    accelgyro.setZGyroOffset(gz_offset);
//
//    meansensors();
//    //Serial.println("...");
//    Serial.print("abs(mean_ax)=");
//    Serial.print(abs(mean_ax));
//    Serial.print(" ay=");
//    Serial.print(abs(mean_ay));
//    Serial.print(" az=");
//    Serial.println(abs(16384-mean_ax));
//
//    if (abs(mean_ax)<=acel_deadzone) ready++;
//    else ax_offset=ax_offset-mean_ax/acel_deadzone;
//
//    if (abs(mean_ay)<=acel_deadzone) ready++;
//    else ay_offset=ay_offset-mean_ay/acel_deadzone;
//
//    if (abs(16384-mean_az)<=acel_deadzone) ready++;
//    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
//
//    if (abs(mean_gx)<=giro_deadzone) ready++;
//    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
//
//    if (abs(mean_gy)<=giro_deadzone) ready++;
//    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);
//
//    if (abs(mean_gz)<=giro_deadzone) ready++;
//    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);
//
//    if (ready==6) break;
//  }
//}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
