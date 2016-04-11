#include <Stepper.h>
#include <Ultrasonic.h>

#define D1 7          // Направление вращение двигателя 1
#define M1 3          // ШИМ вывод для управления двигателем 1
#define D2 4          // Направление вращение двигателя 2
#define M2 5          // ШИМ вывод для управления двигателем 2

#define SPEED 1400 // RPM

//Stepper stepper(20, 9, 11, 8, 10); // 8, 9, 10, 11 - пины
//Ultrasonic ultrasonic(8, 9); // 8, 9 - пины

int gyro_delay_time = 80; 
float delta_drive_power = 0.022 * gyro_delay_time; //2.2
float leftDrivePower = 70;
float rightDrivePower = 70;
float gyro_accuracy = (0.001251428 - 0.0002) * gyro_delay_time; //проверено эмпирически (отклонение за 1мсек = 0.001251428)

int direction = 0;   // Текущее направление вращения     0 - вперед
//                                  1 - назад
int value;            // Текущее значение ШИМ

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void gyroInit();
float checkGyroState();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void driveStop() {
  analogWrite(M1, 0);
  analogWrite(M2, 0);
  delay(5000);
  analogWrite(M1, leftDrivePower);
  analogWrite(M2, rightDrivePower);
}

//bool checkState(int left_b, int right_b) {
//  int cm = ultrasonic.Ranging(CM);
//
//  Serial.print(cm);
//  Serial.println(" cm");
//  if (cm > left_b && cm < right_b)
//    driveStop();
//}

void gyroSetup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(85);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  int setup_time = 0;
  int zero_count = 0;
  while (true){
    if (abs(checkGyroState()) == 0.00)
      zero_count++;
    delay(gyro_delay_time);
    setup_time += gyro_delay_time;    
    gyroInit();
    
    Serial.print("zero_count = ");
    Serial.println(zero_count);
    if(zero_count == 20) // значение получено эмпирически, примерно столько раз попадаются "левые" нули при калибровке 
      break;
  }

//  while (abs(checkGyroState()) != 0.00){
//    delay(gyro_delay_time);
//    setup_time += gyro_delay_time;    
//    gyroInit();
//  }

  if(setup_time < 2000)
    Serial.print("ERROR GYRO CALIBRATE"); 
  else{
    Serial.print("setup time (ms)= ");
    Serial.println(setup_time);
  }
}

void gyroInit() {
  uint8_t fifoBuffer[1024]; // FIFO storage buffer
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    #endif
    
    for (int i = 0; i < 3; i++ )
      ypr[i] = ypr[i] * 180 / M_PI;
  }
}

float checkGyroState() {
  int gyroStatus;
  float temp_yaw = 0; //вправо - +, влево - -

  gyroInit();
  temp_yaw = ypr[0];

  delay(gyro_delay_time);
  gyroInit();
  
  float delta_yaw = abs(temp_yaw) - abs(ypr[0]);
  Serial.print("\n gyro test:\n\t temp_y = ");
  Serial.println(temp_yaw);
  Serial.print("\t ypr[0] = ");
  Serial.println(ypr[0]);
  Serial.print("\t delta_yaw = ");
  Serial.println(delta_yaw);
  return delta_yaw;
}

float deltaPower(float delta_yaw, float dp){
  int res = 0;
  res = dp;
  return res;
}

void rectificationMovement(float delta_yaw) {
  if ( delta_yaw >= gyro_accuracy) {
    if (rightDrivePower < 255)
      rightDrivePower += deltaPower(delta_yaw, delta_drive_power);
    else
      rightDrivePower = 0;
      
    if (leftDrivePower > -255)
      leftDrivePower -= deltaPower(delta_yaw, delta_drive_power);
    else
      rightDrivePower = 0;
  }
  if ( delta_yaw <= -gyro_accuracy) {
    if (rightDrivePower > -255)
      rightDrivePower -= deltaPower(delta_yaw, delta_drive_power);
    else
     rightDrivePower = 0;
     
    if (leftDrivePower < 255)
      leftDrivePower += deltaPower(delta_yaw, delta_drive_power);
    else
      rightDrivePower = 0;
  }
  Serial.print("rightDrivePower = ");
  Serial.println(rightDrivePower);
  Serial.print("leftDrivePower = ");
  Serial.println(leftDrivePower);
}

void setup() {
  // set the speed of the motor to 30 RPMs
  //stepper.setSpeed(SPEED);

  //Serial.begin (9600);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);

  Serial.begin(115200);

  gyroSetup();
}

//void testMotors(float delta_yaw){
//  if (abs(delta_yaw) > 1.1 && abs(delta_yaw) < 1.1){
//    Serial.print("rightDrivePower = ");
//    Serial.println(rightDrivePower);
//    Serial.print("leftDrivePower = ");
//    Serial.println(leftDrivePower);
//  }
//  else{
//    rightDrivePower++;
//  }
//}

void loop() {
  float delta_yaw = 0;   // разница в значениях гироскопа (по параметру yaw) при движении
  //checkState(0, 10);  // проверяем состояние датчика расстояния

  digitalWrite(D1, direction);  // Задаем направление вращения
  digitalWrite(D2, direction);
  analogWrite(M1, leftDrivePower);       // Задаем скорость вращения
  analogWrite(M2, rightDrivePower);

  delta_yaw = checkGyroState();
//  testMotors(delta_yaw);
//  Serial.print("test delta_yaw power = ");
//  Serial.println(delta_yaw);
  //stepper.step(150);
  //delay(30);

  //stepper.step(-150);
  //delay(30);

  rectificationMovement(delta_yaw);
//  delay(2000);
//  driveStop();
}
