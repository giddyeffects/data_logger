/* ============================================
* Motorcycle Data logger sketch by Gideon Nyaga...
* giddyeffects@gmail.com
===============================================
*/
#include <avr/pgmspace.h>            // for memory storage in program space

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//include time lib
#include "Time.h"
//include RTC lib
#include "DS1307RTC.h"
//include accelerometer lib
#include "MPU6050_6Axis_MotionApps20.h"
#include <elapsedMillis.h> //load the elasped library
elapsedMillis timeElapsed;//Create an Instance

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//include I2C LCD display libary
#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);  // Set the LCD I2C address 16x02 screen
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7,3, POSITIVE);  // Set the LCD I2C address 20x04 screen

//welcome message
char welcomeMsg[] = "HONDA";
char welcomeMsg2[] = "FALCON";
// RTC non-volatile storage
#define RTC_I2C_ADDR  0x68         // Real time clock address on the I2C bus
#define RAM_ADDR      8            // RTC RAM registers start at address 8
#define TRIPA_ADDR    RAM_ADDR     // Trips, ODO, lat and long are all floats, 4 bytes ea.     
#define SERVICE_ADDR    TRIPA_ADDR + 4    //@TODO can change this to service distance
#define ODO_ADDR      SERVICE_ADDR + 4
//#define LAT_ADDR      ODO_ADDR + 4 //tobe activated once i get GPS chip
//#define LON_ADDR      LAT_ADDR + 4 //

//values to store in NVRAM
float f_TripA = 0.0;
float f_Service = 0.0;
float f_ODO = 0.0;

//define & init screens
byte curScreen = 1; //current LCD screen
byte maxScreens = 3; //max screens to be shown
//define other variables
int engRPM = 0; //engine rpm
float airTemp = 0.0;
float engineTemp = 0.0;

//--------------speed & odometer variables----------------------------------------------------

long odometer = 0L; //odometer internal: 0 to 999,999
float tripmeterA = 0.0; //tripmeter internal: 0 to 999
float servicemeter = 0.0; //distance since last service...
String bigKPH;
// wheel radius in mm * 100 * 2 * ( pi * 10000 ) = 94.248000 mm perimeter.
// 6 0's were used in scaling up radius and pi, 6 places are divided in the end
// and the units work out. You can use integers more accurate than float on
// Arduino at greatly faster speed. Both type of long can hold any 9-digits.
// Arduino variable type long long can hold any 19 digits is 19 place accuracy.
// if you work in Small Units and scale back later, integers are plenty accurate.
// remember, this value has to be divided by microseconds per turn.
const unsigned long wheel_perimeter = 1500UL * 2UL * 31416UL; // = 94,248,000 //radius * 2 * pi, 'UL' to force the constant into an unsigned long constant
// wheel perimeter gets divided by microseconds, 1,000,000/sec (usec or us).
// wheel turns once for 94248000 mm/100 in 1000000 usecs = 
//wheel radius is 15mm

unsigned long Speed = 0;
unsigned long PrevSpeed = 0;

volatile byte hall_rising = 0; // interrupt flag
volatile unsigned long irqMicros;

unsigned long startMicros;
unsigned long differenceTimeMicros;

unsigned long hallEffectCount = 0;
unsigned long distance = 0;
//--------------------------------------------------------------------------------------------

//----------- Battery Voltage variables ------------------------------------------------------
// number of analog samples to take per reading
#define NUM_SAMPLES 10
float battVoltage = 0.0; //calculated voltage
int vsum = 0; //sum of samples taken
unsigned char sample_count = 0; // current sample number
//--------------------------------------------------------------------------------------------
int tempWhole, tempFract;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
MPU6050 mpu(0x69); // <-- use for AD0 high using 0x69 coz of RTC which is on 0x68

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
#define HALL_INTERRUPT_PIN 3 //also pin 3 can be used as interrupt
#define LED_PIN 12 // have changed to 12 for now (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BACKLIGHT_PIN 13
#define RPM_LED_PIN 11
#define BAT_LED_PIN 10
#define GPS_LED_PIN 9
#define BATTERY_PIN A0

//define trip meter reset button
#define resetTripBtn 6
//define buttons to operate LCD screen
#define goRightBtn 5  // pushbutton 3 pin
#define goLeftBtn 4  // pushbutton 4 pin
#define airTempPin A3 //using analog pin 3 to measure sensor's input SIGNAL

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

//volatile bool hallInterrupt = false; //indicates whether HALL interrupt pin has gone low??
//my interrupt detection for hall sensor
void magnetDetect(){//called whenever a magnet/interrupt is detected by the hall sensor
  irqMicros = micros();
  hall_rising = 1;
  hallEffectCount++;
}

// ================================================================
// ===               BIG FONT CHARACTER SET                     ===
// ================================================================
const char custom[][8] PROGMEM = {                        // Custom character definitions
      { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00 }, // char 1 
      { 0x18, 0x1C, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }, // char 2 
      { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x07, 0x03 }, // char 3 
      { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F }, // char 4 
      { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1E, 0x1C, 0x18 }, // char 5 
      { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x1F }, // char 6 
      { 0x1F, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F }, // char 7 
      { 0x03, 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F }  // char 8 
};
const char bigChars[][8] PROGMEM = {
      { 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // Space
      { 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // !
      { 0x05, 0x05, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00 }, // "
      { 0x04, 0xFF, 0x04, 0xFF, 0x04, 0x01, 0xFF, 0x01 }, // #
      { 0x08, 0xFF, 0x06, 0x07, 0xFF, 0x05, 0x00, 0x00 }, // $
      { 0x01, 0x20, 0x04, 0x01, 0x04, 0x01, 0x20, 0x04 }, // %
      { 0x08, 0x06, 0x02, 0x20, 0x03, 0x07, 0x02, 0x04 }, // &
      { 0x05, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // '
      { 0x08, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00 }, // (
      { 0x01, 0x02, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00 }, // )
      { 0x01, 0x04, 0x04, 0x01, 0x04, 0x01, 0x01, 0x04 }, // *
      { 0x04, 0xFF, 0x04, 0x01, 0xFF, 0x01, 0x00, 0x00 }, // +
      { 0x20, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // 
      { 0x04, 0x04, 0x04, 0x20, 0x20, 0x20, 0x00, 0x00 }, // -
      { 0x20, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // .
      { 0x20, 0x20, 0x04, 0x01, 0x04, 0x01, 0x20, 0x20 }, // /
      { 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00 }, // 0
      { 0x01, 0x02, 0x20, 0x04, 0xFF, 0x04, 0x00, 0x00 }, // 1
      { 0x06, 0x06, 0x02, 0xFF, 0x07, 0x07, 0x00, 0x00 }, // 2
      { 0x01, 0x06, 0x02, 0x04, 0x07, 0x05, 0x00, 0x00 }, // 3
      { 0x03, 0x04, 0xFF, 0x20, 0x20, 0xFF, 0x00, 0x00 }, // 4
      { 0xFF, 0x06, 0x06, 0x07, 0x07, 0x05, 0x00, 0x00 }, // 5
      { 0x08, 0x06, 0x06, 0x03, 0x07, 0x05, 0x00, 0x00 }, // 6
      { 0x01, 0x01, 0x02, 0x20, 0x08, 0x20, 0x00, 0x00 }, // 7
      { 0x08, 0x06, 0x02, 0x03, 0x07, 0x05, 0x00, 0x00 }, // 8
      { 0x08, 0x06, 0x02, 0x07, 0x07, 0x05, 0x00, 0x00 }, // 9
      { 0xA5, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // :
      { 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, // ;
      { 0x20, 0x04, 0x01, 0x01, 0x01, 0x04, 0x00, 0x00 }, // <
      { 0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00 }, // =
      { 0x01, 0x04, 0x20, 0x04, 0x01, 0x01, 0x00, 0x00 }, // >
      { 0x01, 0x06, 0x02, 0x20, 0x07, 0x20, 0x00, 0x00 }, // ?
      { 0x08, 0x06, 0x02, 0x03, 0x04, 0x04, 0x00, 0x00 }, // @
      { 0x08, 0x06, 0x02, 0xFF, 0x20, 0xFF, 0x00, 0x00 }, // A
      { 0xFF, 0x06, 0x05, 0xFF, 0x07, 0x02, 0x00, 0x00 }, // B
      { 0x08, 0x01, 0x01, 0x03, 0x04, 0x04, 0x00, 0x00 }, // C
      { 0xFF, 0x01, 0x02, 0xFF, 0x04, 0x05, 0x00, 0x00 }, // D
      { 0xFF, 0x06, 0x06, 0xFF, 0x07, 0x07, 0x00, 0x00 }, // E
      { 0xFF, 0x06, 0x06, 0xFF, 0x20, 0x20, 0x00, 0x00 }, // F
      { 0x08, 0x01, 0x01, 0x03, 0x04, 0x02, 0x00, 0x00 }, // G
      { 0xFF, 0x04, 0xFF, 0xFF, 0x20, 0xFF, 0x00, 0x00 }, // H
      { 0x01, 0xFF, 0x01, 0x04, 0xFF, 0x04, 0x00, 0x00 }, // I
      { 0x20, 0x20, 0xFF, 0x04, 0x04, 0x05, 0x00, 0x00 }, // J
      { 0xFF, 0x04, 0x05, 0xFF, 0x20, 0x02, 0x00, 0x00 }, // K
      { 0xFF, 0x20, 0x20, 0xFF, 0x04, 0x04, 0x00, 0x00 }, // L
      { 0x08, 0x03, 0x05, 0x02, 0xFF, 0x20, 0x20, 0xFF }, // M
      { 0xFF, 0x02, 0x20, 0xFF, 0xFF, 0x20, 0x03, 0xFF }, // N
      { 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00 }, // 0
      { 0x08, 0x06, 0x02, 0xFF, 0x20, 0x20, 0x00, 0x00 }, // P
      { 0x08, 0x01, 0x02, 0x20, 0x03, 0x04, 0xFF, 0x04 }, // Q
      { 0xFF, 0x06, 0x02, 0xFF, 0x20, 0x02, 0x00, 0x00 }, // R
      { 0x08, 0x06, 0x06, 0x07, 0x07, 0x05, 0x00, 0x00 }, // S
      { 0x01, 0xFF, 0x01, 0x20, 0xFF, 0x20, 0x00, 0x00 }, // T
      { 0xFF, 0x20, 0xFF, 0x03, 0x04, 0x05, 0x00, 0x00 }, // U
      { 0x03, 0x20, 0x20, 0x05, 0x20, 0x02, 0x08, 0x20 }, // V
      { 0xFF, 0x20, 0x20, 0xFF, 0x03, 0x08, 0x02, 0x05 }, // W
      { 0x03, 0x04, 0x05, 0x08, 0x20, 0x02, 0x00, 0x00 }, // X
      { 0x03, 0x04, 0x05, 0x20, 0xFF, 0x20, 0x00, 0x00 }, // Y
      { 0x01, 0x06, 0x05, 0x08, 0x07, 0x04, 0x00, 0x00 }, // Z
      { 0xFF, 0x01, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00 }, // [
      { 0x01, 0x04, 0x20, 0x20, 0x20, 0x20, 0x01, 0x04 }, // Backslash
      { 0x01, 0xFF, 0x04, 0xFF, 0x00, 0x00, 0x00, 0x00 }, // ]
      { 0x08, 0x02, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00 }, // ^
      { 0x20, 0x20, 0x20, 0x04, 0x04, 0x04, 0x00, 0x00 }  // _
};
byte col,row,nb=0,bc=0;                                   // general
byte bb[8];                                               // byte buffer for reading from PROGMEM

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

    //lcd.begin(16,2);               // initialize the lcd for 16x02
    lcd.begin(20,4);               // initialize the lcd for 20x04
    for (nb=0; nb<8; nb++ ) {                     // create 8 custom characters
      for (bc=0; bc<8; bc++) bb[bc]= pgm_read_byte( &custom[nb][bc] );
      lcd.createChar ( nb+1, bb );
    }
    lcd.home ();                   // go home
    writeBigString(welcomeMsg, 0, 0);
    writeBigString(welcomeMsg2, 0, 2);
    //lcd.print(welcomeMsg);        //welcome message
    delay(1500);
    lcd.clear();
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(HALL_INTERRUPT_PIN, INPUT);
    Serial.begin(9600);//@TODO remove later on
    
    // the function to get the time from the RTC
    setSyncProvider(RTC.get);

    //  Read trip, ODO, and location data from RTC storage to init vals
    f_TripA = RTC_ReadFloat( TRIPA_ADDR );
    f_Service = RTC_ReadFloat( SERVICE_ADDR );
    f_ODO   = RTC_ReadFloat( ODO_ADDR );

    tripmeterA = f_TripA;
    servicemeter = f_Service;
    odometer  = f_ODO;
    
  //  KEEP THIS - Needed to set the odometer
    //f_ODO = 5577.0f;
    //odometer = f_ODO;
    //RTC_WriteFloat( ODO_ADDR, f_ODO );
    
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

    //attach magnet interrupt
    attachInterrupt(digitalPinToInterrupt(HALL_INTERRUPT_PIN), magnetDetect, FALLING);

    // configure LEDs for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(RPM_LED_PIN, OUTPUT);
    pinMode(BAT_LED_PIN, OUTPUT);
    pinMode(GPS_LED_PIN, OUTPUT);
}


// ********************************************************************************** //
//                                      SUBROUTINES
// ********************************************************************************** //
// writeBigChar: writes big character 'ch' to column x, row y; returns number of columns used by 'ch'
int writeBigChar(char ch, byte x, byte y) {
  if (ch < ' ' || ch > '_') return 0;               // If outside table range, do nothing
  nb=0;                                             // character byte counter 
  for (bc=0; bc<8; bc++) {                        
    bb[bc] = pgm_read_byte( &bigChars[ch-' '][bc] );  // read 8 bytes from PROGMEM
    if(bb[bc] != 0) nb++;
  }  
 
  bc=0;
  for (row = y; row < y+2; row++) {
    for (col = x; col < x+nb/2; col++ ) {
      lcd.setCursor(col, row);                      // move to position
      lcd.write(bb[bc++]);                          // write byte and increment to next
    }
//    lcd.setCursor(col, row);
//    lcd.write(' ');                                 // Write ' ' between letters
  }
  return nb/2-1;                                      // returns number of columns used by char
}

// writeBigString: writes out each letter of string
void writeBigString(char *str, byte x, byte y) {
  char c;
  while ((c = *str++))
  x += writeBigChar(c, x, y) + 1;
}

//-----get temp in degrees Celsius-----
float getTemp(int pin){
  float temp;
  temp = analogRead(pin)*4.65/1023.0;//max output arduino can read is 5V and ADC is 10bit add .0 to make it float..note USB doesn't give 5V so put 4.6V in the meantime
  temp = temp - 0.4;//0 degC is 0.4V
  temp = temp / 0.01953;//for MCP9701A 0.01953V per degC
  return temp;
}
//not using this function but it's here just in case
void getTemp2(int pin){
  int ADVal = analogRead(pin), Tc_100 = 25 * ADVal - 2050;
  tempWhole = Tc_100 / 100;
  tempFract = Tc_100 % 100;
}

//----process screen stuff----
void processScreens(){
  //my stuff about screens here
  int rightButtonState = digitalRead(goRightBtn);
  if (rightButtonState == LOW){ //go to screen on the right
    lcd.clear(); //clear screen only if changing the screen
    if(timeElapsed > 500) { //allow for switch debounce check if this is necessary...
      curScreen = curScreen + 1;
      if(curScreen > maxScreens){//reset screens when max is reached
        curScreen = 1;
      }
      timeElapsed = 0;
    }
  }

  int leftButtonState = digitalRead(goLeftBtn);
  if (leftButtonState == LOW ){ //go to screen on the left
    lcd.clear(); //clear screen only if changing the screen
    if(timeElapsed > 500) {
      curScreen = curScreen - 1;
      if (curScreen < 1){
        curScreen = maxScreens;
      }
      timeElapsed = 0;
    }
  }
}

void processTripReset(){
  int tripResetState = digitalRead(resetTripBtn);
  if (tripResetState == LOW){
    //elapsedMillis timeElapsed2;
    //if(timeElapsed2 > 3000) {
      delay(1200);
      tripResetState = digitalRead(resetTripBtn);
      if(tripResetState == LOW){ //@TODO reset service after holding button for say 10 seconds
        tripmeterA = 0.0;
      }
    //}
  }
}
void processVoltage(){
  // take a number of analog samples and add them up
  while (sample_count < NUM_SAMPLES) {
      vsum += analogRead(BATTERY_PIN);
      sample_count++;
      //delay(10);
  }
  // calculate the voltage
  // use 5.0 for a 5.0V ADC reference voltage
  // 5.015V is the calibrated reference voltage
  battVoltage = ((float)sum / (float)NUM_SAMPLES * 5.015) / 1024.0;
  // send voltage for display on Serial Monitor
  // voltage multiplied by 11 when using voltage divider that
  // divides by 11. 11.132 is the calibrated voltage divide
  // value
  sample_count = 0;
  vsum = 0;
}
void processSpeed() {
//  hallState = digitalRead(HALL_INTERRUPT_PIN);
//  if (hallState == LOW) {
//    revs = revs + 1;
//  }
while(hall_rising == 1){
      differenceTimeMicros = irqMicros - startMicros;
      startMicros = irqMicros;
      hall_rising = 0;
    }
  
    distance = (wheel_perimeter * hallEffectCount)/1000000000; //distance in meters
    if( differenceTimeMicros != 0 ){
      Speed =  wheel_perimeter / differenceTimeMicros; //speed = distance / time
      Speed = (Speed*3600)/1000000; // this converts the speed from mm/s to Km/h
      
      if ( Speed != PrevSpeed )
      {
        Serial.print( distance );
        Serial.println( "mm  " );
        //Serial.println( differenceMicros ); // this now shows mm/sec with no remainder
        Serial.print( Speed ); // this converts the speed from mm/s to Km/h
        //Serial.print( "mm/s  alt=" );
        Serial.print( "KM/H  " );
        Serial.println( " " );
  
        
      }
    
    }else {
      Speed = 0;
      Serial.print( Speed ); 
      //Serial.print( "mm/s  alt=" );
      Serial.println( "KM/H  " );
    }
    
    

    PrevSpeed = Speed;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.print('0');
  }
  lcd.print(number);
}

void showTime(){
  //set time elements
  tmElements_t tm;
//show the time if RTC is OK
  if (RTC.read(tm)) {
    print2digits(tm.Hour);
    lcd.print(":");
    print2digits(tm.Minute);
  }
  else { 
    if (RTC.chipPresent()) {
      lcd.print("setTime");  
    }
    else { lcd.print("TmCheck");}
  }
}
//**************************
// RTC DS1307
// NVRAM register access functions
//

// Reads a 2-byte integer value from the RTC RAM registers
int RTC_ReadInteger( int valAddr )
{
  // Set the register pointer
  Wire.beginTransmission( RTC_I2C_ADDR );
  Wire.write( valAddr );
  Wire.endTransmission( );

  // Read 2 bytes into int value
  Wire.requestFrom( RTC_I2C_ADDR, 2 );
  int value = Wire.read( );
  value = ( value << 8 ) + Wire.read( );
  
  return value;
}

// Reads a 4-byte float value from the RTC RAM registers
float RTC_ReadFloat( int valAddr )
{
  float value;
  byte  *byteArray = (byte *) &value;
  
  // Set the register pointer
  Wire.beginTransmission( RTC_I2C_ADDR );
  Wire.write( valAddr );
  Wire.endTransmission( );

  // Read 4 bytes can convert to float value
  Wire.requestFrom( RTC_I2C_ADDR, 4 );
  byteArray[3] = Wire.read( );
  byteArray[2] = Wire.read( );
  byteArray[1] = Wire.read( );
  byteArray[0] = Wire.read( );
  
  return value;
}

// Writes a 2-byte integer value to the RTC RAM registers
void RTC_WriteInteger( int valAddr, int value )
{
  if( valAddr > 7 && valAddr  < 63 )  // Don't let writes go to the RTC registers 0 - 7
  {
    Wire.beginTransmission( RTC_I2C_ADDR );
    Wire.write( valAddr );

    // Write high byte, low byte
    Wire.write( ( unsigned char )( value >> 8 ) );
    Wire.write( ( unsigned char )value ); 
    
    Wire.endTransmission( );
  }  
}

// Writes a 4-byte float value to the RTC RAM registers
void RTC_WriteFloat( int valAddr, float value )
{
  if( valAddr > 7 && valAddr  < 61 )  // Don't let writes go to the RTC registers 0 - 7
  {
    Wire.beginTransmission( RTC_I2C_ADDR );
    Wire.write( valAddr );
    
    // Write high word (high byte/low byte), low word (high byte/low byte)
    byte  *byteArray;
    byteArray = (byte *) &value;
    Wire.write( byteArray[3] );
    Wire.write( byteArray[2] );
    Wire.write( byteArray[1] );
    Wire.write( byteArray[0] );

    Wire.endTransmission( );
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
    processSpeed();
    /*while(!hallInterrupt){
      tripmeterA = tripmeterA + 1;
    }
    hallInterrupt = false;*/
    
    processScreens();
    //show relevant info the different screens
    switch (curScreen) {
        case 1: {
          lcd.home ();                   // go home
          //lcd.clear();                   //clear screen
          //writeBigString(revs, 0, 0);
          if(Speed < 10){ bigKPH = "00"; bigKPH += Speed ; }
          if (Speed >= 10 && Speed < 100){  bigKPH = "0"; bigKPH += Speed ; }
          char c[bigKPH.length()+1]; bigKPH.toCharArray(c, bigKPH.length()+1);
          writeBigString(c, 0, 0);
          lcd.print("kmph");
          lcd.setCursor ( 13, 0 );
          showTime(); // :-)
          lcd.setCursor ( 0, 2 );        // go to the next line
          lcd.print("rpm:");
          //lcd.print(engRPM);
          lcd.print("9999");
          lcd.print(" ");
          lcd.print("BAT:");
          lcd.print("14.53");
          //lcd.print(battVoltage);
          lcd.print("V");
          lcd.setCursor ( 0, 3 );        // go to the next line
          lcd.print("Odo:");
          lcd.print(odometer);
          //lcd.print("999999");
          lcd.print(" ");
          lcd.print ("TripA:");
          //lcd.print(tripmeterA);
          lcd.print("999");
          break;
        }

        case 2: {
          lcd.home ();                   // go home
          //lcd.clear();                   //clear screen
          lcd.print("AirTemp:");//air Temp
          lcd.print(airTemp);
          lcd.print((char)223); //degree sign
          lcd.print("C TRIP");
          lcd.setCursor ( 0, 1 );        // go to the next line
          lcd.print("EngTemp:");//engine Temp
          lcd.print(engineTemp);
          lcd.print((char)223); //degree sign
          lcd.print("C  B");
          lcd.setCursor ( 0, 2 );        // go to the next line
          lcd.print("LAT:");
          lcd.setCursor(15,2);
          lcd.print(":9999");
          lcd.setCursor ( 0, 3 );        // go to the next line
          lcd.print ("LONG:");
          break;
        }

         case 3: {
          //lcd.clear();
          lcd.home ();
          lcd.print(" ========<>======== ");
          lcd.setCursor ( 0, 1 );        
          lcd.print("Yaw:");
          lcd.print(ypr[0] * 180/M_PI);
          lcd.setCursor ( 0, 2 );        // go to the next line
          lcd.print("Pitch:");
          lcd.print(ypr[1] * 180/M_PI);
          lcd.setCursor ( 0, 3 );
          lcd.print("Roll:");
          lcd.print(ypr[2] * 180/M_PI);
          break;
         }
    }
}
// ********************************************************************************** //
//                                      OPERATION ROUTINES
// ********************************************************************************** //
// FREERAM: Returns the number of bytes currently free in RAM  
int freeRam(void) {
  extern int  __bss_end, *__brkval; 
  int free_memory; 
  if((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__bss_end); 
  } 
  else {
    free_memory = ((int)&free_memory) - ((int)__brkval); 
  }
  return free_memory; 
}
