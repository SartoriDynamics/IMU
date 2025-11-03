// I2C libray communication
#include <Wire.h>

// IMU I2C address
#define MPU   0x68

float theta = 0.0, theta_speed = 0.0;
float ax_angle = 0;
float gx_angle = 0;
float gx_float = 0;

float Ts = 0.01, currentT = 0.0, previousT = 0.0;        // Elapsed time in loop() function

float Q_angle = 0.0088; // Angular data confidence
float Q_bias  = 0.0088; // Angular velocity data confidence
float R_meas  = 0.0305;
float roll_angle = 0.0;
float bias = 0.0;
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float K[2] = {0, 0};

// MAIN SETUP
void setup() { // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);                   // make sure your Serial Monitor is also set at this baud rate.
  IMUsetup();
  currentT = millis();

  pinMode (2, OUTPUT);
  pinMode (0, INPUT);
  while(digitalRead(0) == HIGH); //Wait for BOOT buton
}

// MAIN LOOP
void loop() {// put your main code here, to run repeatedly:

  currentT = millis();
  if ((currentT - previousT)/1000.0 >= Ts) {
    previousT = currentT;

    IMUread();
    
    Serial.print(currentT/1000.0, 6);
    Serial.print(",");
    Serial.print(ax_angle, 6);
    Serial.print(",");
    Serial.print(gx_angle, 6);
    Serial.print(",");
    Serial.print(theta, 6);
    Serial.print(",");
    Serial.print(bias, 6);
    Serial.print(",");
    Serial.println(gx_float, 6);

    digitalWrite(2, HIGH);
  }

}

// SETUP functions
void IMUsetup(){
  // Initialize the MPU6050
  Wire.beginTransmission(MPU);           // begin, Send the slave address (in this case 68)              
  Wire.write(0x6B);                      // make the reset (write 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);            // end the transmission

  // Gyro config
  Wire.beginTransmission(MPU);           // begin, Send the slave address (in this case 68)
  Wire.write(0x1B);                      // We want to write to the GYRO_CONFIG register (0x1B)
  Wire.write(0x00000000);                // 0x00000000 -> FS_SEL = 0 (±250 °/s)
  Wire.endTransmission();                // End the transmission with the gyro

  // Acc config
  Wire.beginTransmission(MPU);           // Start communication with the address found during search
  Wire.write(0x1C);                      // We want to write to the ACCEL_CONFIG register
  Wire.write(0b00000000);                // Set to ±2g full scale range
  Wire.endTransmission();
}

// IMU function: Kalman Filter
void IMUread(){
  // read IMU
  int16_t ax,ay,az,temp,gx,gy,gz;
  Wire.beginTransmission(MPU);    // IMU address: 0x68
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);        // IMU address: 0x68
  ax=Wire.read()<<8|Wire.read();   // X-axis value: 16384.0; 
  ay=Wire.read()<<8|Wire.read();   // Y-axis value: 16384.0;     
  az=Wire.read()<<8|Wire.read();   // Z-axis value: 16384.0;  
  temp=Wire.read()<<8|Wire.read();      
  gx=Wire.read()<<8|Wire.read();  
  gy=Wire.read()<<8|Wire.read();  
  gz=Wire.read()<<8|Wire.read();  
  //accelerometer angles in degrees (or rads)
  ax_angle = atan2(ay, sqrt(ax*ax + az*az)) * 57.3; // roll
  //float ay_angle = atan2(ax, sqrt(ay*ay + az*az)) * 57.3; // pitch
  // float az_angle = atan2(sqrt(ax*ax + az*az), az) * 57.3; // yaw (useless)
  // gyro measurements in degress (or rads)
  gx_float =  gx / 131.0; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gy =  gy / 131.0; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gz =  gz / 131.0; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  
  // begin: Kalman filter - Roll Axis (X)
  roll_angle += (gx_float - bias) * Ts;
  
  P[0][0] += ( (P[1][1]*Ts) - P[0][1] - P[1][0] + (Q_angle * Ts) ) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias;
  //
  K[0] = P[0][0] / (P[0][0] + R_meas);
  K[1] = P[1][0] / (P[0][0] + R_meas);  
  //
  roll_angle += K[0] * (ax_angle - roll_angle); 
  bias       += K[1] * (ax_angle - roll_angle);
  //
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  // end: Kalman filter 

  theta_speed = gx_float - bias; // Unbiased gyro speed
  gx_angle += gx_float * Ts;
  theta = roll_angle;

}
