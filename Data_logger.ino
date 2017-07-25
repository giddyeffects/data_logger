/* ============================================
* Motorcycle Data logger sketch by Gideon Nyaga...
* giddyeffects@gmail.com
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//include time lib
#include "Time.h"
//include RTC lib
#include "DS1307RTC.h"
//include accelerometer lib
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//include I2C LCD display libary
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);  // Set the LCD I2C address

//welcome message
char welcomeMsg[] = "--Honda Falcon--";

//define trip meter reset button
const int resetTripBtn = 5;

const int airTempPin = 3; //using analog pin 3 to measure sensor's input SIGNAL

//define & init screens
int curScreen = 1; //current LCD screen
int maxScreens = 5; //max screens to be shown
//define other variables
int rpm = 0;
long odometer = 0;
float tripmeter = 0.00;
int servicemeter = 0;
float battVoltage = 0.0;
float airTemp = 0.0;
float engineTemp = 0.0;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 12 // have changed to 12 for now (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BACKLIGHT_PIN 13
#define RPM_LED_PIN 11
#define BAT_LED_PIN 10
#define GPS_LED_PIN 9
//define buttons to operate LCD screen
#define goRightBtn 4  // pushbutton 3 pin
#define goLeftBtn 3  // pushbutton 4 pin

bool blinkState = false; //accel LED blinkState

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//@TODO remove variables that won't be used in production data logger
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  //LCD stuff
  //set pushbuttons to be inputs
  pinMode(resetTripBtn, INPUT);
  pinMode(goLeftBtn, INPUT);
  pinMode(goRightBtn, INPUT);

  // Switch on the LCD backlight @TODO can add a pushbutton later on
  pinMode ( BACKLIGHT_PIN, OUTPUT );
  digitalWrite ( BACKLIGHT_PIN, HIGH );

  lcd.begin(16,2);               // initialize the lcd
  lcd.clear();                   //clear screen initially
  lcd.home ();                   // go home
  lcd.print(welcomeMsg);        //welcome message
  delay(1000);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    bool accConnectOk = mpu.testConnection()?true:false;

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready

        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)

        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // configure LEDs for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(RPM_LED_PIN, OUTPUT);
    pinMode(BAT_LED_PIN, OUTPUT);
    pinMode(GPS_LED_PIN, OUTPUT);
}
//-----get temp in degrees Celsius-----
float getTemp(int pin){
  float temp;
  temp = analogRead(pin)*5/1024.0;//max output arduino can read is 5V and ADC is 10bit add .0 to make it float
  temp = temp - 0.4;//0 degC is 0.4V
  temp = temp / 0.01953;//for MCP9701A 0.01953V per degC
  return temp;
}
//----process screen stuff----
void processScreens(){
  //my stuff about screens here
  int rightButtonState = digitalRead(goRightBtn);
  if (rightButtonState == LOW){ //go to screen on the right
    delay(500);//to allow switch debounce
    curScreen = curScreen + 1;
    if(curScreen > maxScreens){//reset screens when max is reached
      curScreen = 1;
    }
  }

  int leftButtonState = digitalRead(goLeftBtn);
  if (leftButtonState == LOW ){ //go to screen on the left
    delay(500);
    curScreen = curScreen - 1;
    if (curScreen < 1){
      curScreen = maxScreens;
    }
  }
}

void processTripReset(){
  int tripResetState = digitalRead(resetTripBtn);
  if (tripResetState == LOW){
    delay(1200);
    tripResetState = digitalRead(resetTripBtn);
    if(tripResetState == LOW){
      tripmeter = 0.0;
    }
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            /*Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);*/
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            /*Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);*/
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            /*Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);*/
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            /*Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);*/
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
    //get air temperature
    airTemp = getTemp(airTempPin);
    processTripReset();
    
    processScreens();
    //show relevant info the different screens
    switch (curScreen) {
        case 1: {
          lcd.home ();                   // go home
          lcd.clear();                   //clear screen
          lcd.print("O:");
          lcd.print(odometer);
          lcd.print(" ");
          lcd.print(rpm);
          lcd.print("rpm");
          lcd.setCursor ( 0, 1 );        // go to the next line
          lcd.print ("A:");
          lcd.print(tripmeter);
          lcd.print(" 999km/h");
          break;
        }

        case 2: {
          lcd.home ();                   // go home
          lcd.clear();                   //clear screen
          lcd.print("AT:");//air Temp
          lcd.print(airTemp);
          lcd.print("C ");
          lcd.print("ET:");//engine Temp
          lcd.print(engineTemp);
          lcd.print("C");
          lcd.setCursor ( 0, 1 );        // go to the next line
          lcd.print ("BAT:");
          lcd.print(battVoltage);
          lcd.print("V 12:59H");
          break;
        }

        case 3: {
          lcd.home ();                   // go home
          lcd.clear();                   //clear screen
          lcd.print("LAT:");
          lcd.setCursor ( 0, 1 );        // go to the next line
          lcd.print ("LONG:");
          break;
        }

         case 4: {
          lcd.clear();
          lcd.home();
          lcd.print("Y:");
          lcd.print(ypr[0] * 180/M_PI);
          lcd.print(" P:");
          lcd.print(ypr[1] * 180/M_PI);
          lcd.setCursor ( 0, 1 );
          lcd.print("Roll:");
          lcd.print(ypr[2] * 180/M_PI);
          break;
         }
        case 5: {
          lcd.clear();
          lcd.home();
          break;
        }
    }
}
