q q/*==============================================================================================GYRO BASLANGIC===================================================================================================
*/
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
/*
   =================================================================================GYRO BITIS==============================================================================================================
*/

/*
   ==================================================================================RFID ==================================================================================================================
*/
#include <RFID.h>
#include <SPI.h>

RFID lrt720(9, 8);

/*
   =====================================================================================RFID FINISHED =====================================================================================================
*/

/*
   ======================================================================================MOTOR=============================================================================================================
*/
#define vel_motor_esq 10
#define vel_motor_dir 11
#define e1 4
#define e2 3
#define d1 12
#define d2 7
/*
   ======================================================================================MOTOR FINISHED ====================================================================================================
*/
double degree;
int currentDegree;
int currentDegree2;
int currentDegree3;
int currentDegree4;
int currentDegree5;
int masaNumarasi;
int masaNumarasi2;
int bitis;

int baslangic = 254;
const int masa1 = 221;
const int masa2 = 66;
const int masa3 = 4;
const int masa4 = 59;
const int masa5 = 120;
const int masa6 = 34;
const int masa7 = 40;

bool ileri = false;
bool teksefer1 = false;
bool teksefer2 = false;
bool teksefer3 = false;
void setup() {
  /*
     ==================================================================================GYRO SETUP=============================================================================================================
  */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);
  /*
     ======================================================================================GYRO SETUP FINISHED ================================================================================================
  */

  /*
     ========================================================================================RFID SETUP========================================================================================================
  */
  SPI.begin();
  lrt720.init();
  pinMode(5, OUTPUT);
  /*
     ============================================================================================RFID SETUP FINISHED ===========================================================================================
  */

  /*===============================================================================================MOTOR SETUP===================================================================================================
  */

  pinMode(vel_motor_esq, OUTPUT);
  pinMode(vel_motor_dir, OUTPUT);

  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);

  /*
     =============================================================================================MOTOR SETUP FINISHED ============================================================================================
  */
}

void loop() {

  /*
     ===========================================================================================GYRO LOOP======================================================================================================
  */

  if (!dmpReady) return;

  while (fifoCount < packetSize) {

    fifoCount = mpu.getFIFOCount();
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

    mpu.resetFIFO();

  } else if (mpuIntStatus & 0x02) {

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_YAWPITCHROLL

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


#endif

    /*
       =============================================================================================GYRO LOOP FINISHED ============================================================================================
    */

    /*
       =================================================================================================RFID LOOP=================================================================================================
    */

    if (lrt720.isCard()) {

      if (lrt720.readCardSerial()) {
        Serial.println(lrt720.serNum[2]);
      }
      lrt720.halt();
    }

    /*
       ==================================================================================================RFID LOOP FINISHED =======================================================================================
    */

    /*
       ==================================================================================================MOTOR LOOP =============================================================================================
    */
    analogWrite(vel_motor_esq, 0);
    analogWrite(vel_motor_dir, 0);
    analogWrite(e1, 0);
    analogWrite(e2, 0);
    analogWrite(d1, 0);
    analogWrite(d2, 0);

    /*
       ===================================================================================================MOTOR LOOP FINISHED ====================================================================================
    */

    /*
       ====================================================================================================PROGRAM FLOW==========================================================================================
    */

    int(degree) = int((ypr[0] * 180 / M_PI) + 360) % 360;

    Serial.print("Degree: ");
    Serial.println(degree);

    Serial.print("CurrentDegree : ");
    Serial.println(currentDegree);

    if (Serial.available()) {
      masaNumarasi = Serial.read();
    }

    switch (masaNumarasi) {
      case 48: Serial.println("Aşçı Masası");
        masaNumarasi2 = 0;
        break;
      case 49: Serial.println("1 Numaralı Masa");
        masaNumarasi2 = 1;
        break;
      case 50: Serial.println("2 Numaralı Masa");
        masaNumarasi2 = 2;
        break;
      case 51: Serial.println("3 Numaralı Masa");
        masaNumarasi2 = 3;
        break;
      case 52: Serial.println("4 Numaralı Masa");
        masaNumarasi2 = 4;
        break;
      case 53: Serial.println("5 Numaralı Masa");
        masaNumarasi2 = 5;
        break;
      case 54: Serial.println("6 Numaralı Masa");
        masaNumarasi2 = 6;
        break;
        default: masaNumarasi2 =6;
    }

    switch (masaNumarasi2) {
      case 0: bitis = 254; ;
        break;
      case 1: bitis = 221;
        break;
      case 2: bitis = 66;
        break;
      case 3: bitis = 4;
        break;
      case 4: bitis = 58;
        break;
      case 5: bitis = 120;
        break;
      case 6: bitis = 34;
        break;
    }


     if (lrt720.serNum[2] == bitis) {

        analogWrite(vel_motor_esq, 255);
        analogWrite(vel_motor_dir, 255);

        analogWrite(e1, 255);
        analogWrite(e2, 255);
        analogWrite(d1, 255);
        analogWrite(d2, 255);

        Serial.println("Siparişiniz Hazır");
        digitalWrite(5, HIGH);
        delay(10000);
        digitalWrite(5, LOW);
        currentDegree = degree;
        lrt720.serNum[2] = 0;
      }

    if (lrt720.serNum[2] == baslangic) {


      Serial.println("Başlangıç");
      digitalWrite(5, HIGH);
      delay(250);
      digitalWrite(5, LOW);
      currentDegree = degree;
      currentDegree2 = currentDegree;
      ileri = true;
      lrt720.serNum[2] = 0;
    }
    if (ileri == true) {
      if (degree < (currentDegree2 - 5)) {
        Serial.println("Hafif Sağ");
        dirS();

      }

      else if (degree > (currentDegree2 + 5)) {
        Serial.println("Hafif Sol");
        esqS();
      }

      else if ((currentDegree2 - 5) < degree < (currentDegree2 + 5)) {
        analogWrite(vel_motor_esq, 120);
        analogWrite(vel_motor_dir, 90);

        analogWrite(e1, 150);
        analogWrite(e2, 0);
        analogWrite(d1, 0);
        analogWrite(d2, 150);

        Serial.println("Düz");
      }
    }

   

    // if (lrt720.serNum[2] == masa5 || lrt720.serNum[2] == masa7 || lrt720.serNum[2] == masa2) {
    if (lrt720.serNum[2] == masa3 || lrt720.serNum[2] == masa4 || lrt720.serNum[2] == masa6 || lrt720.serNum[2] == masa5) {


      if (lrt720.serNum[2] == masa3) {
        if (teksefer1 == false) {
          if (currentDegree2 < 90) {
            currentDegree3 = (90 - currentDegree2);
            currentDegree3 = (360 - currentDegree3);

          }
          else {
            currentDegree3 = currentDegree2 - 90;
          }
        }
        currentDegree2 = currentDegree3;
        teksefer1 = true;
      }
      if (lrt720.serNum[2] == masa4) {
        if (teksefer2 == false) {
          if (currentDegree3 < 90) {
            currentDegree4 = (90 - currentDegree3);
            currentDegree4 = (360 - currentDegree4);
          }
          else {
            currentDegree4 = currentDegree3 - 90;
          }
          currentDegree2 = currentDegree4;
          teksefer2 = true;
        }
      }
      if (lrt720.serNum[2] == masa6) {
        if (teksefer3 == false) {
          if (currentDegree4 < 90) {
            currentDegree5 = (90 - currentDegree4);
            currentDegree5 = (360 - currentDegree5);
          }
          else {
            currentDegree5 = currentDegree4 - 90;
          }
          currentDegree2 = currentDegree3;
          teksefer3 = true;

        }
      }


      ileri = false;

      if (currentDegree < 75) {
        currentDegree += 315;
      }
      else {
        currentDegree = currentDegree;
      }


      if (degree > (currentDegree - 48)) { //============================================================================90' SOLA DONME ===============================================================================
        Serial.println("Girdi");

        esq();

        if (!dmpReady) return;

        while (fifoCount < packetSize) {

          fifoCount = mpu.getFIFOCount();
        }

        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

          mpu.resetFIFO();

        } else if (mpuIntStatus & 0x02) {

          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          mpu.getFIFOBytes(fifoBuffer, packetSize);

          fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_YAWPITCHROLL

          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
        }

      }

      else if (degree < (currentDegree - 95)) { //============================================================================90* Sola Dönme Yardımcı Sağa dönme===========================================================================

        Serial.println("Girdi Sağ");
        dir();

        if (!dmpReady) return;

        while (fifoCount < packetSize) {

          fifoCount = mpu.getFIFOCount();
        }

        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

          mpu.resetFIFO();

        } else if (mpuIntStatus & 0x02) {

          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          mpu.getFIFOBytes(fifoBuffer, packetSize);

          fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_YAWPITCHROLL

          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
        }

      }

      else if ((currentDegree - 95) < degree < (currentDegree - 55)) {  //======================================================================================90*==================================================================
        analogWrite(vel_motor_esq, 150);
        analogWrite(vel_motor_dir, 150);

        analogWrite(e1, 255);
        analogWrite(e2, 255);
        analogWrite(d1, 255);
        analogWrite(d2, 255);
        ileri = true;
        delay(3000);
        analogWrite(e1, 255);
        analogWrite(e2, 0);
        analogWrite(d1, 0);
        analogWrite(d2, 255);
        delay(500);

        lrt720.serNum[2] = 0;

      }
      Serial.println("Köşe Kartı");

    }

    /*
      ============================================================================================END OF FLOW====================================================================================================
    */
  }
}

void esq()
{

  analogWrite(vel_motor_esq, 125);
  analogWrite(vel_motor_dir, 125);

  analogWrite(e1, 0);
  analogWrite(e2, 190);
  analogWrite(d1, 0);
  analogWrite(d2, 190);

}

void dir()
{
  analogWrite(vel_motor_esq, 125);
  analogWrite(vel_motor_dir, 125);

  analogWrite(e1, 190);
  analogWrite(e2, 0);
  analogWrite(d1, 190);
  analogWrite(d2, 0);
}

void esqS()
{

  analogWrite(vel_motor_esq, 145);
  analogWrite(vel_motor_dir, 190);

  analogWrite(e1, 255);
  analogWrite(e2, 0);
  analogWrite(d1, 0);
  analogWrite(d2, 255);

}

void dirS()
{
  analogWrite(vel_motor_esq, 190);
  analogWrite(vel_motor_dir, 145);

  analogWrite(e1, 255);
  analogWrite(e2, 0);
  analogWrite(d1, 0);
  analogWrite(d2, 255);
}
