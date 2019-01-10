/* ============================================
*   Motorcycle Data logger sketch by Gideon Nyaga...
*   giddyeffects@gmail.com
*  ===============================================
*/

//==========INCLUDES==========
// for memory storage in program space
#include <avr/pgmspace.h>
//I2C Bus lib
#include "I2Cdev.h"
//include time lib
#include "Time.h"
#undef DAYS_PER_WEEK 
//include RealTimeClock/RTC lib
#include "DS1307RTC.h"
//load the elasped library
#include <elapsedMillis.h>
//Arduino Wire Library
#include "Wire.h"
//include I2C LCD display libary
#include <LiquidCrystal_I2C.h>
//include NEOGPS library
#include <NMEAGPS.h>
//include the GPSPort library
#include <GPSport.h>

//==========VARIABLES==========
//Init LCD
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //=> addr, EN, RW, RS, D4, D5, D6, D7, BacklightPin, POLARITY... Set the LCD I2C address 20x04 screen
//Create an elapsedMillis Instance
elapsedMillis timeElapsed;
//create GPS variable
NMEAGPS gps; //this parses the GPS characters
gps_fix fix; //this holds on to the latest values
bool cansettime = true;
double gps_lat = 0.00, gps_long = 0.00;
//welcome message
char welcomeMsg[] = "HONDA", welcomeMsg2[] = "FALCON";
// RTC non-volatile storage
#define RTC_I2C_ADDR 0x68           // Real time clock address on the I2C bus
#define RAM_ADDR 8                  // RTC RAM registers start at address 8
#define TRIPA_ADDR RAM_ADDR         // Trips, ODO, lat and long are all floats, 4 bytes ea.
#define SERVICE_ADDR TRIPA_ADDR + 4 //@TODO can change this to tripmeter B
#define ODO_ADDR SERVICE_ADDR + 4
//#define LAT_ADDR      ODO_ADDR + 4 //tobe activated once i get GPS chip
//#define LON_ADDR      LAT_ADDR + 4 //

//==========PIN ALLOCATION==========
#define INTERRUPT_PIN 2      // use pin 2 on Arduino Uno & most boards
#define HALL_INTERRUPT_PIN 3 //also pin 3 can be used as interrupt
#define LED_PIN 13           //(Arduino is 13, Teensy is 11, Teensy++ is 6)
#define GPS_LED_PIN 12
#define RPM_LED_PIN 11
#define BAT_LED_PIN 10
//pins 9 and 8 used for the software serial
#define SERVICE_LED_PIN 7
#define BATTERY_PIN A2 //use analog pin2 to read motorbike battery voltage
//define trip meter reset button
#define resetTripBtn 6
//define buttons to operate LCD screen
#define backlightBtn 5 // pushbutton 5 pin
#define goRightBtn 4   // pushbutton 4 pin
#define tempPin A3  //using analog pin 3 to measure temp sensor's input SIGNAL


//values to store in NVRAM
float f_TripA = 0.0, f_Service = 0.0, f_ODO = 0.0;

//==========SPEED & ODO VARIABLES==========
long odometer = 0L;       //odometer internal: 0 to 999,999
float tripmeterA = 0.0;   //tripmeter internal: 0 to 999
float servicemeter = 0.0; //distance since last service...
String bigKPH;
int rotationcounter = 0, vspeed = 0, dvspeed = 0, tyrec = 1935; //tyre circumference in ??
long deltat = 0;
long timea = 0;
long timec = 0;

int speedrr = 500;
const int speedarraysize = 5;
int speedarray[speedarraysize];
int arrayposition = 0;

//==========BATTERY VOLTAGE VARIABES==========
// number of analog samples to take per reading
#define NUM_SAMPLES 10
float battVoltage = 0.0;        //calculated voltage
int vsum = 0;                   //voltage sum of samples taken
unsigned char sample_count = 0; // current sample number

//==========define & init screens==========
byte curScreen = 1;  //current LCD screen
byte maxScreens = 3; //max screens to be shown

//==========define other variables==========
int engRPM = 0; //engine rpm
float airTemp = 0.0;
float engineTemp = 0.0;
bool backlightState = true;

//I2C address of the MPU6050.
const int MPU=0x69;  
const float alpha = 0.955; //Low Pass Filter smoothing factor
int16_t ax,ay,az,gx,gy,gz,airTempMPU;//was int
double pitch, roll, yaw, fXg=0, fYg=0, fZg=0, timer;
double accXangle ,accYangle,accZangle ,gyroXrate ,gyroYrate,gyroZrate;
double gyroXAngle, gyroYAngle, gyroZAngle;

// ================================================================
// ===               BIG FONT CHARACTER SET                     ===
// ================================================================
const char custom[][8] PROGMEM = {
    // Custom character definitions
    {0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00}, // char 1
    {0x18, 0x1C, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}, // char 2
    {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x07, 0x03}, // char 3
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F}, // char 4
    {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1E, 0x1C, 0x18}, // char 5
    {0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x1F}, // char 6
    {0x1F, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F}, // char 7
    {0x03, 0x07, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}  // char 8
};
const char bigChars[][8] PROGMEM = {
    {0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // !
    {0x05, 0x05, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00}, // "
    {0x04, 0xFF, 0x04, 0xFF, 0x04, 0x01, 0xFF, 0x01}, // #
    {0x08, 0xFF, 0x06, 0x07, 0xFF, 0x05, 0x00, 0x00}, // $
    {0x01, 0x20, 0x04, 0x01, 0x04, 0x01, 0x20, 0x04}, // %
    {0x08, 0x06, 0x02, 0x20, 0x03, 0x07, 0x02, 0x04}, // &
    {0x05, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '
    {0x08, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00}, // (
    {0x01, 0x02, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00}, // )
    {0x01, 0x04, 0x04, 0x01, 0x04, 0x01, 0x01, 0x04}, // *
    {0x04, 0xFF, 0x04, 0x01, 0xFF, 0x01, 0x00, 0x00}, // +
    {0x20, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //
    {0x04, 0x04, 0x04, 0x20, 0x20, 0x20, 0x00, 0x00}, // -
    {0x20, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // .
    {0x20, 0x20, 0x04, 0x01, 0x04, 0x01, 0x20, 0x20}, // /
    {0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00}, // 0
    {0x01, 0x02, 0x20, 0x04, 0xFF, 0x04, 0x00, 0x00}, // 1
    {0x06, 0x06, 0x02, 0xFF, 0x07, 0x07, 0x00, 0x00}, // 2
    {0x01, 0x06, 0x02, 0x04, 0x07, 0x05, 0x00, 0x00}, // 3
    {0x03, 0x04, 0xFF, 0x20, 0x20, 0xFF, 0x00, 0x00}, // 4
    {0xFF, 0x06, 0x06, 0x07, 0x07, 0x05, 0x00, 0x00}, // 5
    {0x08, 0x06, 0x06, 0x03, 0x07, 0x05, 0x00, 0x00}, // 6
    {0x01, 0x01, 0x02, 0x20, 0x08, 0x20, 0x00, 0x00}, // 7
    {0x08, 0x06, 0x02, 0x03, 0x07, 0x05, 0x00, 0x00}, // 8
    {0x08, 0x06, 0x02, 0x07, 0x07, 0x05, 0x00, 0x00}, // 9
    {0xA5, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // :
    {0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ;
    {0x20, 0x04, 0x01, 0x01, 0x01, 0x04, 0x00, 0x00}, // <
    {0x04, 0x04, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00}, // =
    {0x01, 0x04, 0x20, 0x04, 0x01, 0x01, 0x00, 0x00}, // >
    {0x01, 0x06, 0x02, 0x20, 0x07, 0x20, 0x00, 0x00}, // ?
    {0x08, 0x06, 0x02, 0x03, 0x04, 0x04, 0x00, 0x00}, // @
    {0x08, 0x06, 0x02, 0xFF, 0x20, 0xFF, 0x00, 0x00}, // A
    {0xFF, 0x06, 0x05, 0xFF, 0x07, 0x02, 0x00, 0x00}, // B
    {0x08, 0x01, 0x01, 0x03, 0x04, 0x04, 0x00, 0x00}, // C
    {0xFF, 0x01, 0x02, 0xFF, 0x04, 0x05, 0x00, 0x00}, // D
    {0xFF, 0x06, 0x06, 0xFF, 0x07, 0x07, 0x00, 0x00}, // E
    {0xFF, 0x06, 0x06, 0xFF, 0x20, 0x20, 0x00, 0x00}, // F
    {0x08, 0x01, 0x01, 0x03, 0x04, 0x02, 0x00, 0x00}, // G
    {0xFF, 0x04, 0xFF, 0xFF, 0x20, 0xFF, 0x00, 0x00}, // H
    {0x01, 0xFF, 0x01, 0x04, 0xFF, 0x04, 0x00, 0x00}, // I
    {0x20, 0x20, 0xFF, 0x04, 0x04, 0x05, 0x00, 0x00}, // J
    {0xFF, 0x04, 0x05, 0xFF, 0x20, 0x02, 0x00, 0x00}, // K
    {0xFF, 0x20, 0x20, 0xFF, 0x04, 0x04, 0x00, 0x00}, // L
    {0x08, 0x03, 0x05, 0x02, 0xFF, 0x20, 0x20, 0xFF}, // M
    {0xFF, 0x02, 0x20, 0xFF, 0xFF, 0x20, 0x03, 0xFF}, // N
    {0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00}, // 0
    {0x08, 0x06, 0x02, 0xFF, 0x20, 0x20, 0x00, 0x00}, // P
    {0x08, 0x01, 0x02, 0x20, 0x03, 0x04, 0xFF, 0x04}, // Q
    {0xFF, 0x06, 0x02, 0xFF, 0x20, 0x02, 0x00, 0x00}, // R
    {0x08, 0x06, 0x06, 0x07, 0x07, 0x05, 0x00, 0x00}, // S
    {0x01, 0xFF, 0x01, 0x20, 0xFF, 0x20, 0x00, 0x00}, // T
    {0xFF, 0x20, 0xFF, 0x03, 0x04, 0x05, 0x00, 0x00}, // U
    {0x03, 0x20, 0x20, 0x05, 0x20, 0x02, 0x08, 0x20}, // V
    {0xFF, 0x20, 0x20, 0xFF, 0x03, 0x08, 0x02, 0x05}, // W
    {0x03, 0x04, 0x05, 0x08, 0x20, 0x02, 0x00, 0x00}, // X
    {0x03, 0x04, 0x05, 0x20, 0xFF, 0x20, 0x00, 0x00}, // Y
    {0x01, 0x06, 0x05, 0x08, 0x07, 0x04, 0x00, 0x00}, // Z
    {0xFF, 0x01, 0xFF, 0x04, 0x00, 0x00, 0x00, 0x00}, // [
    {0x01, 0x04, 0x20, 0x20, 0x20, 0x20, 0x01, 0x04}, // Backslash
    {0x01, 0xFF, 0x04, 0xFF, 0x00, 0x00, 0x00, 0x00}, // ]
    {0x08, 0x02, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00}, // ^
    {0x20, 0x20, 0x20, 0x04, 0x04, 0x04, 0x00, 0x00}  // _
};
byte col, row, nb = 0, bc = 0; // general
byte bb[8];                    // byte buffer for reading from PROGMEM

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

//interrupt detection for hall sensor
void magnetDetect() { //called whenever a magnet/interrupt is detected by the hall sensor
  vspeed = tyrec * 3.6 / (millis() - deltat);
  timea = millis();
  deltat = millis();
  rotationcounter = rotationcounter + 1;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  //set pushbuttons to be inputs
  pinMode(resetTripBtn, INPUT);
  pinMode(backlightBtn, INPUT);
  pinMode(goRightBtn, INPUT);
  //set interrupt pins to be inputs
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(HALL_INTERRUPT_PIN, INPUT);

  lcd.begin(20, 4); // initialize the lcd for 20x04
  for (nb = 0; nb < 8; nb++) { // create 8 custom characters
    for (bc = 0; bc < 8; bc++)
      bb[bc] = pgm_read_byte(&custom[nb][bc]);
    lcd.createChar(nb + 1, bb);
  }
  lcd.home(); // go home
  writeBigString(welcomeMsg, 0, 0);
  writeBigString(welcomeMsg2, 0, 2);
  delay(1500);
  lcd.clear();
  // the function to get the time from the RTC
  setSyncProvider(RTC.get);
  //  Read trip, ODO, and location data from RTC storage to init vals
  f_TripA = RTC_ReadFloat(TRIPA_ADDR);
  f_Service = RTC_ReadFloat(SERVICE_ADDR);
  f_ODO = RTC_ReadFloat(ODO_ADDR);

  tripmeterA = f_TripA;
  servicemeter = f_Service;
  odometer = f_ODO;

  //  KEEP THIS - Needed to set the odometer
  //f_ODO = 5577.0f;
  //odometer = f_ODO;
  //RTC_WriteFloat( ODO_ADDR, f_ODO );

  //attach magnet interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_INTERRUPT_PIN), magnetDetect, FALLING);

  // configure LEDs for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(RPM_LED_PIN, OUTPUT);
  pinMode(BAT_LED_PIN, OUTPUT);
  pinMode(GPS_LED_PIN, OUTPUT);
  pinMode(SERVICE_LED_PIN, OUTPUT);
  Wire.begin(); //start the wire :)
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  setupMPU();
  //setup GPS
  gpsPort.begin(9600);
}
// ********************************************************************************** //
//                                      SUBROUTINES
// ********************************************************************************** //

void setupMPU() {
  //=========MPU stuff=========
  Serial.begin(9600);

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  //Requests the data from the sensor.
  Wire.requestFrom(MPU, 14, true);

  //Read the sensor data.
  ax = Wire.read() << 8 | Wire.read();      //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read();      //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();      //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  airTempMPU = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();      //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();      //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();      //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  /*Serial.println("Raw Values");
  Serial.println("==========");
  Serial.print("ax/ay/az/gx/gy/gz/airTempMPU\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.print("\t");
  Serial.print(airTempMPU / 340 + 36.53);
  Serial.println();*/

  accXangle = (atan2(ay, az) * RAD_TO_DEG);
  accYangle = (atan2(ax, az) * RAD_TO_DEG);
  accZangle = (atan2(ax, ay) * RAD_TO_DEG); /* calculate yaw but not correct*/
  gyroXrate = gx / 16.5;
  gyroYrate = gy / 16.5;
  gyroZrate = gz / 16.5;
  timer = millis();
  //angular position
  gyroXAngle += gyroXrate * (millis() - timer) / 1000;
  gyroYAngle += gyroYrate * (millis() - timer) / 1000;
  gyroZAngle += gyroZrate * (millis() - timer) / 1000; /* calculate yaw but not correct*/

  roll = alpha * (roll + gyroXAngle) + (1 - alpha) * accXangle;
  pitch = alpha * (pitch + gyroYAngle) + (1 - alpha) * accYangle;
  yaw = alpha * (yaw + gyroZAngle) + (1 - alpha) * accZangle; /*yaw but not correct*/
    /* first attempt
    //Low Pass filter
    fXg = gx * alpha + (fXg * (1.0 - alpha));
    fYg = gy * alpha + (fYg * (1.0 - alpha));
    fZg = gz * alpha + (fZg * (1.0 - alpha));

    //Roll & Pitch Equations
    roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
    pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI; */
  /*Serial.print(yaw);
  Serial.print(":");
  Serial.print(pitch);
  Serial.print(":");
  Serial.println(roll);*/
  delay(100);
}

// writeBigChar: writes big character 'ch' to column x, row y; returns number of columns used by 'ch'
int writeBigChar(char ch, byte x, byte y)
{
  if (ch < ' ' || ch > '_')
    return 0; // If outside table range, do nothing
  nb = 0;     // character byte counter
  for (bc = 0; bc < 8; bc++) {
    bb[bc] = pgm_read_byte(&bigChars[ch - ' '][bc]); // read 8 bytes from PROGMEM
    if (bb[bc] != 0)
      nb++;
  }

  bc = 0;
  for (row = y; row < y + 2; row++) {
    for (col = x; col < x + nb / 2; col++) {
      lcd.setCursor(col, row); // move to position
      lcd.write(bb[bc++]);     // write byte and increment to next
    }
    //    lcd.setCursor(col, row);
    //    lcd.write(' ');                                 // Write ' ' between letters
  }
  return nb / 2 - 1; // returns number of columns used by char
}

// writeBigString: writes out each letter of string
void writeBigString(char *str, byte x, byte y) {
  char c;
  while ((c = *str++))
    x += writeBigChar(c, x, y) + 1;
}

//-----get temp in degrees Celsius-----
float getTemp(int pin) {
  float temp;
  temp = analogRead(pin) * 4.65 / 1023.0; //max output arduino can read is 5V and ADC is 10bit add .0 to make it float..note USB doesn't give 5V so put 4.6V in the meantime
  temp = temp - 0.4;                      //0 degC is 0.4V
  temp = temp / 0.01953;                  //for MCP9701A 0.01953V per degC
  return temp;
}

//----process screen stuff----
void processScreens() {
  //my stuff about screens here
  int rightButtonState = digitalRead(goRightBtn);
  if (rightButtonState == LOW) {              
    //go to screen on the right
    lcd.clear(); //clear screen only if changing the screen
    if (timeElapsed > 500) { //allow for switch debounce check if this is necessary...
      curScreen = curScreen + 1;
      if (curScreen > maxScreens) { 
        //reset screens when max is reached
        curScreen = 1;
      }
      timeElapsed = 0;
    }
  }

  //@NOTE: not using a left button for now..changed it to backlight button
  //  int leftButtonState = digitalRead(goLeftBtn);
  //  if (leftButtonState == LOW ){ //go to screen on the left
  //    lcd.clear(); //clear screen only if changing the screen
  //    if(timeElapsed > 500) {
  //      curScreen = curScreen - 1;
  //      if (curScreen < 1){
  //        curScreen = maxScreens;
  //      }
  //      timeElapsed = 0;
  //    }
  //  }
}

void processBacklightBtn() {
  int bbtnState = digitalRead(backlightBtn);
  if (bbtnState == LOW) {
    if (timeElapsed > 500) { //allow for switch debounce check if this is necessary...
      //toggle backlight
      backlightState = !backlightState;
      lcd.setBacklight(backlightState);
      timeElapsed = 0;
    }
  }
}

void processTripReset() {
  int tripResetState = digitalRead(resetTripBtn);
  if (tripResetState == LOW) {
    //elapsedMillis timeElapsed2;
    //if(timeElapsed2 > 3000) {
    delay(1200);
    tripResetState = digitalRead(resetTripBtn);
    if (tripResetState == LOW) { 
      //@TODO reset service after holding button for say 10 seconds
      f_TripA = 0.0;
      tripmeterA = f_TripA;
      RTC_WriteFloat(TRIPA_ADDR, f_TripA);
    }
    //}
  }
}

void processVoltage() {
  // take a number of analog samples and add them up
  while (sample_count < NUM_SAMPLES) {
    vsum += analogRead(BATTERY_PIN);
    sample_count++;
    //delay(10);
  }
  // calculate the voltage
  // use 5.0 for a 5.0V ADC reference voltage
  // 5.015V is the calibrated reference voltage
  battVoltage = ((float)vsum / (float)NUM_SAMPLES * 5.015) / 1024.0;
  // send voltage for display on Serial Monitor
  // voltage multiplied by 11 when using voltage divider that
  // divides by 11. 11.132 is the calibrated voltage divide
  // value
  sample_count = 0;
  vsum = 0;
}

void processSpeed() {
  if (millis() - timea > 1000) {
    vspeed = 0;
    timea = millis();
  }

  if (millis() - timec >= speedrr / speedarraysize) {
    speedarray[arrayposition] = vspeed;
    arrayposition = arrayposition + 1;

    if (arrayposition == speedarraysize) {
      arrayposition = 0;
      dvspeed = 0;

      for (int x = 0; x < speedarraysize; x++) {
        dvspeed = dvspeed + speedarray[x];
      }

      /*
       * The following line calculates the speed in km/h.
       * If you want the speed in MPH change it into this:
       * dvspeed=(dvspeed/speedarraysize)*0.621371192;
       */
      dvspeed = dvspeed / speedarraysize;
    }
    timec = millis();
  }

  //actions after driving 1km @TODO change to every 100m for tripmeter
  if (rotationcounter >= 1000000 / tyrec) {
    f_ODO = f_ODO + 1;
    f_TripA = f_TripA + 1;
    f_Service = f_Service + 1;
    odometer = f_ODO;
    tripmeterA = f_TripA;
    servicemeter = f_Service;

    if (tripmeterA > 999) {
      tripmeterA = 0;
    }
    // Save trip, ODO and Service floats to RTC storage
    RTC_WriteFloat(TRIPA_ADDR, f_TripA);
    RTC_WriteFloat(ODO_ADDR, f_ODO);
    RTC_WriteFloat(SERVICE_ADDR, f_Service);
    rotationcounter = 0;
  }
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.print('0');
  }
  lcd.print(number);
}

void showTime() {
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
      //set time from the GPS
      lcd.print("setTime");
    }
    else {
      lcd.print("TmCheck");
    }
  }
}

//**************************
// RTC DS1307
// NVRAM register access functions
//**************************

// Reads a 2-byte integer value from the RTC RAM registers
int RTC_ReadInteger(int valAddr) {
  // Set the register pointer
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(valAddr);
  Wire.endTransmission();

  // Read 2 bytes into int value
  Wire.requestFrom(RTC_I2C_ADDR, 2);
  int value = Wire.read();
  value = (value << 8) + Wire.read();

  return value;
}

// Reads a 4-byte float value from the RTC RAM registers
float RTC_ReadFloat(int valAddr) {
  float value;
  byte *byteArray = (byte *)&value;

  // Set the register pointer
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(valAddr);
  Wire.endTransmission();

  // Read 4 bytes can convert to float value
  Wire.requestFrom(RTC_I2C_ADDR, 4);
  byteArray[3] = Wire.read();
  byteArray[2] = Wire.read();
  byteArray[1] = Wire.read();
  byteArray[0] = Wire.read();

  return value;
}

// Writes a 2-byte integer value to the RTC RAM registers
void RTC_WriteInteger(int valAddr, int value) {
  if (valAddr > 7 && valAddr < 63) { // Don't let writes go to the RTC registers 0 - 7
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(valAddr);

    // Write high byte, low byte
    Wire.write((unsigned char)(value >> 8));
    Wire.write((unsigned char)value);

    Wire.endTransmission();
  }
}

// Writes a 4-byte float value to the RTC RAM registers
void RTC_WriteFloat(int valAddr, float value) {
  if (valAddr > 7 && valAddr < 61) { // Don't let writes go to the RTC registers 0 - 7
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(valAddr);

    // Write high word (high byte/low byte), low word (high byte/low byte)
    byte *byteArray;
    byteArray = (byte *)&value;
    Wire.write(byteArray[3]);
    Wire.write(byteArray[2]);
    Wire.write(byteArray[1]);
    Wire.write(byteArray[0]);

    Wire.endTransmission();
  }
}

// ================================================================
// ===                    GPS FUNCTIONS                    ===
// ================================================================
void getGPS() {
  while (gps.available( gpsPort )) {
    fix = gps.read();
    //@TODO confirm if this is the right check!!! or is it fix.valid.status??
    if(fix.valid.location){
      //if gps location locked..turn on GPS LED
      digitalWrite(GPS_LED_PIN, HIGH);
      gps_lat = fix.latitude();
      gps_long = fix.longitude();
      DEBUG_PORT.print(fix.latitude(), 6);
      DEBUG_PORT.print(',');
      DEBUG_PORT.print(fix.longitude(), 6);
      DEBUG_PORT.println();
    }
    else {
      gps_lat = 0.00;
      gps_long = 0.00;
      DEBUG_PORT.println("No location fix yet");
      digitalWrite(GPS_LED_PIN, LOW);
    }
    //
    if(fix.valid.speed){
      DEBUG_PORT.print("Speed(kph):");
      DEBUG_PORT.println(fix.speed_kph());
    }
    if(fix.valid.time && cansettime) {
      cansettime = false;//set time only once

    }
    
    //digitalWrite(GPS_LED_PIN, !digitalRead(GPS_LED_PIN)); // toggle the GPS_LED_PIN
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  //get air temperature
  engineTemp = getTemp(tempPin);//using the temp sensor
  //check and process backlight button
  processBacklightBtn();
  //check and process trip reset button
  processTripReset();
  //calculate speed
  processSpeed();

  processScreens();
  //show relevant info the different screens
  switch (curScreen) {
    case 1: {
      lcd.home(); // go home
      //lcd.clear();                   //clear screen
      //writeBigString(revs, 0, 0);
      if (dvspeed < 10) {
        bigKPH = "00";
        bigKPH += dvspeed;
      }
      if (dvspeed >= 10 && dvspeed < 100) {
        bigKPH = "0";
        bigKPH += dvspeed;
      }
      char c[bigKPH.length() + 1];
      bigKPH.toCharArray(c, bigKPH.length() + 1);
      writeBigString(c, 0, 0);
      lcd.print("kmph");
      lcd.setCursor(13, 0);
      showTime();          // :-)
      lcd.setCursor(0, 2); // go to the next line
      lcd.print("rpm:");
      //lcd.print(engRPM);
      lcd.print("9999");
      lcd.print(" ");
      lcd.print("BAT:");
      lcd.print("14.53");
      //lcd.print(battVoltage);
      lcd.print("V");
      lcd.setCursor(0, 3); // go to the next line
      lcd.print("Odo:");
      lcd.print(odometer);
      //lcd.print("999999");
      lcd.print(" ");
      lcd.print("TRP:");
      lcd.print(tripmeterA);
      //lcd.print("999.9");
      break;
    }

    case 2: {
      lcd.home(); // go home
      //lcd.clear();                   //clear screen
      lcd.print("AirTemp:"); //air Temp
      lcd.print(airTempMPU / 340 + 36.53);//air temp accg to the MPU
      lcd.print((char)223); //degree sign
      lcd.print("C TRIP");
      lcd.setCursor(0, 1);   // go to the next line
      lcd.print("EngTemp:"); //engine Temp
      lcd.print(engineTemp);
      lcd.print((char)223); //degree sign
      lcd.print("C  B");
      lcd.setCursor(0, 2); // go to the next line
      lcd.print("LAT:");
      lcd.print(gps_lat,5);//print the latitude
      lcd.setCursor(15, 2);
      lcd.print(":9999");
      lcd.setCursor(0, 3); // go to the next line
      lcd.print("LONG:");
      lcd.print(gps_long,5);//print the longitude
      break;
    }

    case 3: {
      //lcd.clear();
      lcd.home();
      lcd.print(" ========<>======== ");
      lcd.setCursor(0, 1);
      lcd.print("Yaw:");
      lcd.print(yaw);
      lcd.setCursor(0, 2); // go to the next line
      lcd.print("Pitch:");
      lcd.print(pitch);
      lcd.setCursor(0, 3);
      lcd.print("Roll:");
      lcd.print(roll);
      break;
    }
  }
  readMPU();
  getGPS();
}
// ********************************************************************************** //
//                                      OPERATION ROUTINES
// ********************************************************************************** //
// FREERAM: Returns the number of bytes currently free in RAM
int freeRam(void) {
  extern int __bss_end, *__brkval;
  int free_memory;
  if ((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }
  else {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}
