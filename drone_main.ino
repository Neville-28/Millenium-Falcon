#include<Servo.h>                                           //include servo library
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
float anglegx,anglegy,anglegz;
float anglex,angley,anglez;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long acc_total_vector;
float angle_pitch_acc,angle_roll_acc;
Servo speedcontrolur;                                        //declare motors
Servo speedcontrolul;                                        //-
Servo speedcontroldr;                                        //-
Servo speedcontroldl;                                        //-

int speedul =1000;
int speedur =1000;
int speeddl =1000;
int speeddr =1000;
int ur;                                                       //value for feed back
int ul;                                                       //-
int dr;                                                       //-
int dl;                                                       //-
void setup() {
  speedcontrolur.attach(8);                                  //assign motors to pins
  speedcontrolul.attach(9);                                  //-
  speedcontroldr.attach(10);                                 //-
  speedcontroldl.attach(11);                                 //-

  setupMPU();
  Serial.begin(9600);
   Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1);
  }
   mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
   mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
 
    for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //calibrate the gyro by find the offset
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyro_x_cal += gx;                                              
    gyro_y_cal += gy;                                              
    gyro_z_cal += gz;                                              
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
     gyro_x_cal /= 2000;                                                  
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;
  }
    speedcontrolur.writeMicroseconds(speedur);                   //intialize the speed of motor
  speedcontrolul.writeMicroseconds(speedur);
  speedcontroldr.writeMicroseconds(speeddr);
  speedcontroldl.writeMicroseconds(speeddl);
  
}

void loop() {
  ur =0;
  ul = 0;
  dr = 0;
  dl=0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
gx -= gyro_x_cal;                                               
  gy -= gyro_y_cal;                                                
  gz -= gyro_z_cal; 
anglegx+=gx/250/65.5*2;                                       //angle with gyro
anglegy+=gy/250/65.5*2;
anglegz+=gz/250/65.5*2;
  anglegx += anglegy * sin(gz * 0.000001066*2);               //If the IMU rotated in x axis transfer the roll angle to the pitch angel
  anglegy -= anglegx * sin(gz * 0.000001066*2);               //If the IMU rotated in the pitch angle to the roll angel
    acc_total_vector = sqrt((ax*ax)+(ay*ay)+(az*az));  //find the angle with accel
 
  angle_pitch_acc = asin((float)ay/acc_total_vector)* 57.296;     
  angle_roll_acc = asin((float)ax/acc_total_vector)* -57.296;  
  anglex = 0.996*anglegx +0.004* angle_pitch_acc;    //remove the error of the drift of gyro by combine the angle of gyro and angle of accel
  angley = 0.996*anglegx +0.004*angle_roll_acc;
delay(3);
  if (Serial.available() > 0) {
    int command = Serial.read();
    Serial.print('k');
    if (command == 'u' && speedur < 2700) {
      speedur = speedur + 3;
      speedul = speedul + 3;
      speeddr = speeddr + 3;
      speeddl = speeddl + 3;
    }
    if (command == 'd' && speedur > 100) {
      speedur = speedur - 4;
      speedul = speedul - 4;
      speeddr = speeddr - 4;
      speeddl = speeddl - 4;
    }
    if(anglex>0){
    dl = -15*anglex;
    ul=-15*anglex;
    }
    else if(anglex<0){
      dr = 15*anglex;
      dl = 15*anglex;
    }
    if(angley>0){
      ul=-angley*15;     
      ur=-angley*15;
    }
    else if(angley<0){
      dl=angley*15;
      dr=angley*15;
    }
    
  }
  speedcontrolur.writeMicroseconds(speedur+ur);
  speedcontrolul.writeMicroseconds(speedul+ul);
  speedcontroldr.writeMicroseconds(speeddr+dr);
  speedcontroldl.writeMicroseconds(speeddl+dl);



}
  void setupMPU(){

  Wire.beginTransmission(0b1101000); 

  Wire.write(0x6B); 

  Wire.write(0b00000000); 

  Wire.endTransmission();  

  Wire.beginTransmission(0b1101000); //I2C address of the MPU

  Wire.write(0x1B); 

  Wire.write(0x00000011); //Setting the gyro to full scale +/- 500deg./s 

  Wire.endTransmission(); 

  Wire.beginTransmission(0b1101000); //I2C address of the MPU

  Wire.write(0x1C); 

  Wire.write(0b00001000); //Setting the accel to +/- 8g

  Wire.endTransmission(); 

}



void recordAccelRegisters() {

  Wire.beginTransmission(0b1101000); //I2C address of the MPU

  Wire.write(0x3B); //Starting register for Accel Readings

  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)

  while(Wire.available() < 6);

  ax = Wire.read()<<8|Wire.read(); //get ax

  ay = Wire.read()<<8|Wire.read(); //get ay

  az = Wire.read()<<8|Wire.read(); //get az



}







void recordGyroRegisters() {

  Wire.beginTransmission(0b1101000); //I2C address of the MPU

  Wire.write(0x43); //Starting register for Gyro Readings

  Wire.endTransmission();

  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)

  while(Wire.available() < 6);

  gx = Wire.read()<<8|Wire.read(); //get gx

  gy = Wire.read()<<8|Wire.read(); //get gy

  gz = Wire.read()<<8|Wire.read(); //get gz

}






