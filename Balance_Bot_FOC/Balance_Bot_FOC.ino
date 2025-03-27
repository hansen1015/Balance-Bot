#include <SimpleFOC.h>

BLDCMotor motorL = BLDCMotor(7);
BLDCDriver3PWM driverL = BLDCDriver3PWM(32,33,25,22);

// BLDC motorL & driverL instance
BLDCMotor motorR = BLDCMotor(7);
BLDCDriver3PWM driverR  = BLDCDriver3PWM(26,27,14,12);

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14];

double Output, Set_angle, Error, Last_E, Inte;
double Kp=1.2, Ki=0, Kd=0.0001;

double speedL = 0, speedR = 0;

bool Motor_enable = 0, L_enable = 1;

void setup() {
  driverL.voltage_power_supply = 12;
  driverL.init();
  motorL.linkDriver(&driverL);
  motorL.KV_rating = 260;
  motorL.voltage_limit = 4;   // [V]
  motorL.velocity_limit = 40; // [rad/s]
  
  driverR.voltage_power_supply = 12;
  driverR.init();
  motorR.linkDriver(&driverR);
  motorR.KV_rating = 260;
  motorR.voltage_limit = 4;   // [V]
  motorR.velocity_limit = 40; // [rad/s]

 
  //Open loop control mode setting
  motorL.controller = MotionControlType::velocity_openloop;
  motorR.controller = MotionControlType::velocity_openloop;

  //Initialize the hardware
  motorL.init();
  motorR.init();
  
  Serial.begin(115200);
  Wire.begin(19,18);
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  xTaskCreatePinnedToCore(Motor, "Motor", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(IMU, "IMU", 1024, NULL, 1, NULL, 1);
}

void IMU(void *pvParameters){
  while(true){
    while (i2cRead(0x3B, i2cData, 14));
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    //Serial.print(kalAngleX); Serial.print("\t");
    //Serial.print(kalAngleY); Serial.print("\n");
    if(abs(kalAngleY) < 25){
      Error = kalAngleY-Set_angle;
      Inte += Error;
      Output = -(Kp*(kalAngleY)+Ki*Inte/dt+Kd*(Error-Last_E)/dt);
      Last_E = Error;
      Motor_enable = true;
    }
    else{
      Last_E = kalAngleY;
      Inte = 0;
      Output = 0;
      Motor_enable = false;
    }
    speedL = Output;
    speedR = Output;
    //Serial.println(Output);
    vTaskDelay(5);
  }
}

void Motor(void *pvParameters){
  while(true){
    if(Motor_enable != L_enable){
      if(Motor_enable){
        motorL.enable();
        motorR.enable();
      }
      else{
        motorL.disable();
        motorR.disable();
      }
    }
    speedL = constrain(speedL,-40,40);
    motorL.move(speedL);
    speedR = constrain(speedR,-40,40);
    motorR.move(speedR);
    motorL.loopFOC();
    motorR.loopFOC();

    L_enable = Motor_enable;
    vTaskDelay(1);
  }
}


void loop() {
}
