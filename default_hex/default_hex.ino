/* 

  WAS: Factory firmware for HexBright FLEX 
  v2.4  Dec 30, 2012
  
*/

#include <math.h>
#include <Wire.h>

// Settings
#define OVERTEMP                340
// Constants
#define ACC_ADDRESS             0x4C
#define ACC_REG_XOUT            0
#define ACC_REG_YOUT            1
#define ACC_REG_ZOUT            2
#define ACC_REG_TILT            3
#define ACC_REG_INTS            6
#define ACC_REG_MODE            7
// Pin assignments
#define DPIN_RLED_SW            2
#define DPIN_GLED               5
#define DPIN_PGOOD              7
#define DPIN_PWR                8
#define DPIN_DRV_MODE           9
#define DPIN_DRV_EN             10
#define DPIN_ACC_INT            3
#define APIN_TEMP               0
#define APIN_CHARGE             3
// Interrupts
#define INT_SW                  0
#define INT_ACC                 1
// Modes
#define MODE_OFF                0
#define MODE_OFF_PREVIEW        1
#define MODE_AUTO_TILT          2
#define MODE_AUTO_ROLL          3
#define MODE_LOW                4
#define MODE_MED                5
#define MODE_HIGH               6
#define MODE_BLINKING           7
#define MODE_ABLINKING_PREVIEW  8
#define MODE_DAZZLING           9
#define MODE_AUTO_BLINKING      10
#define MODE_AUTO_BLINKING_SET  11
// State
byte mode = 0;
unsigned long btnTime = 0;
boolean btnDown = false;
boolean accelDebug = false;
//sensor smoothing 
const int numReadings = 25;
int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
float smoothedVal;

void setup()
{
  // We just powered on!  That means either we got plugged 
  // into USB, or the user is pressing the power button.
  pinMode(DPIN_PWR,      INPUT);
  digitalWrite(DPIN_PWR, LOW);

  // Initialize GPIO
  pinMode(DPIN_RLED_SW,  INPUT);
  pinMode(DPIN_GLED,     OUTPUT);
  pinMode(DPIN_DRV_MODE, OUTPUT);
  pinMode(DPIN_DRV_EN,   OUTPUT);
  pinMode(DPIN_ACC_INT,  INPUT);
  pinMode(DPIN_PGOOD,    INPUT);
  digitalWrite(DPIN_DRV_MODE, LOW);
  digitalWrite(DPIN_DRV_EN,   LOW);
  digitalWrite(DPIN_ACC_INT,  HIGH);
  // Initialize serial busses
  Serial.begin(9600);
  Wire.begin();
  // Configure accelerometer
  byte config[] = {
    ACC_REG_INTS,  // First register (see next line)
    0xE4,  // Interrupts: shakes, taps
    0x00,  // Mode: not enabled yet
    0x00,  // Sample rate: 120 Hz
    0x0F,  // Tap threshold
    0x10   // Tap debounce samples
  };
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(config, sizeof(config));
  Wire.endTransmission();

  // Enable accelerometer
  byte enable[] = {ACC_REG_MODE, 0x01};  // Mode: active!
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(enable, sizeof(enable));
  Wire.endTransmission();
  btnTime = millis();
  btnDown = digitalRead(DPIN_RLED_SW);
  mode = MODE_OFF;
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0; 
  Serial.println("Powered up!");
}

void loop()
{
  static unsigned long lastTime, lastTempTime, lastAccTime;
  unsigned long time = millis();
  static byte blink, bright_debug;
  static int blinkRate;
  float angle;
  
  // Check the state of the charge controller
  int chargeState = analogRead(APIN_CHARGE);
  if (chargeState < 128)  // Low - charging
  {
    digitalWrite(DPIN_GLED, (time&0x0100)?LOW:HIGH);
  }
  else if (chargeState > 768) // High - charged
  {
    digitalWrite(DPIN_GLED, HIGH);
  }
  else // Hi-Z - shutdown
  {
    digitalWrite(DPIN_GLED, LOW);    
  }
  
  // Check the serial port
  if (Serial.available())
  {
    char c = Serial.read();
    switch (c)
    {
    case 's':
      accelDebug = !accelDebug;
    break;
    case '+':
      bright_debug++;
      analogWrite(DPIN_DRV_EN, bright_debug);
      Serial.println(bright_debug);
    break;
    case '-':
      bright_debug--;
      analogWrite(DPIN_DRV_EN, bright_debug);
      Serial.println(bright_debug);
    break;
    case '*':
      bright_debug = bright_debug + 15;
      analogWrite(DPIN_DRV_EN, bright_debug);
      Serial.println(bright_debug);
    break;
    case '/':
      bright_debug = bright_debug - 15;
      analogWrite(DPIN_DRV_EN, bright_debug);
      Serial.println(bright_debug);
    break;
    case 'p':
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      Serial.println("power on\n");
    break;
    case 'x':
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, LOW);
      Serial.println("power off\n");
    break;
    case 'h':
      digitalWrite(DPIN_DRV_MODE, HIGH);
      Serial.println("high\n");
    break;
    case 'l':
      digitalWrite(DPIN_DRV_MODE, LOW);
      Serial.println("low\n");
    break;
    case 't':
      {
        int temperature = analogRead(APIN_TEMP);
        Serial.print("Temperature = ");
        Serial.println(temperature);
      }
    break;
    case 'w':
      {
        byte pgood = digitalRead(DPIN_PGOOD);
        Serial.print("LED driver power good = ");
        Serial.println(pgood?"Yes":"No");
        Serial.print("chargeState: ");
        Serial.println(chargeState);
      }
    break;
    }
  }
  
  // Check the temperature sensor
  if (time-lastTempTime > 1000)
  {
    lastTempTime = time;
    int temperature = analogRead(APIN_TEMP);
    if (temperature > OVERTEMP && mode != MODE_OFF)
    {
      Serial.println("Overheating!");

      for (int i = 0; i < 6; i++)
      {
        digitalWrite(DPIN_DRV_MODE, LOW);
        delay(100);
        digitalWrite(DPIN_DRV_MODE, HIGH);
        delay(100);
      }
      digitalWrite(DPIN_DRV_MODE, LOW);

      mode = MODE_LOW;
    }
  
  if(accelDebug) // only print this every secound
    {
      char accel[3];
      readAccel(accel);
      Serial.print("Acceleration = ");
      Serial.print(accel[0], DEC);
      Serial.print(", ");
      Serial.print(accel[1], DEC);
      Serial.print(", ");
      Serial.print(accel[2], DEC);
      Serial.print(", ");
      angle = readTiltAngle(0);
      Serial.print(angle, DEC);
      Serial.print(", ");
      angle = readTiltAngle(1);
      Serial.print(angle, DEC);
      Serial.print(", ");
      angle = readTiltAngle(2);
      Serial.println(angle, DEC);
    }
  }

  byte tapped = 0, shaked = 0;
  if (!digitalRead(DPIN_ACC_INT))
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_TILT);
    Wire.endTransmission(false);       // End, but do not stop!
    Wire.requestFrom(ACC_ADDRESS, 1);  // This one stops.
    byte tilt = Wire.read();
    
    if (time-lastAccTime > 500)
    {
      lastAccTime = time;
  
      tapped = !!(tilt & 0x20);
      shaked = !!(tilt & 0x80);
  
      if (tapped) Serial.println("Tap!");
      if (shaked) Serial.println("Shake!");
    }
  }
  
  // Do whatever this mode does
  switch (mode)
  {
  case MODE_AUTO_TILT:
    //sensor smoothing
    total = total - readings[index];         
    // read from the sensor:  
    readings[index] = readTiltAngle(1); // reads tilt
    total = total + readings[index];       
    index = index + 1;                    
    if (index >= numReadings) index = 0;                           
    // calculate the average:
    average = total / numReadings;
    // further smoothing with interagtion from last reading and low pass filter  
    smoothedVal = (average * (1 - 0.5)) + (smoothedVal  *  0.5);
    angle = smoothedVal;
    Serial.print("angle = ");
    Serial.print(angle);
    angle = map(angle, 180, 90, 0, 512);
    Serial.print("\1map = ");
    Serial.print(angle);
    if(angle<20) angle = 20;
    if(angle>255) 
    {
      angle = map(angle, 255, 512, 50, 255);
      if(angle>255) angle = 255;
      analogWrite(DPIN_DRV_EN, angle);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      Serial.print("\t HIG");
    }
    else 
    { 
      digitalWrite(DPIN_DRV_MODE, LOW);
      analogWrite(DPIN_DRV_EN, angle);
      Serial.print("\tLOW ");
    }
    Serial.print("\2map = ");
    Serial.println(angle);
    analogWrite(DPIN_DRV_EN, angle);
  break;
  case MODE_AUTO_ROLL:
      //sensor smoothing
    total = total - readings[index];         
    // read from the sensor:  
    readings[index] = readTiltAngle(2); // 2 reads roll 
    total = total + readings[index];       
    index = index + 1;                    
    if (index >= numReadings) index = 0;                           
    // calculate the average:
    average = total / numReadings;
    // further smoothing with interagtion from last reading and low pass filter  
    smoothedVal = (average * (1 - 0.5)) + (smoothedVal  *  0.5);
    angle = smoothedVal;
    Serial.print("angle = ");
    Serial.print(angle);
    angle = map(angle, 5, 170, 0, 512);
    Serial.print("\1map = ");
    Serial.print(angle);
    if(angle<20) angle = 20;
    if(angle>255) 
    {
      angle = map(angle, 255, 512, 50, 255);
      if(angle>255) angle = 255;
      analogWrite(DPIN_DRV_EN, angle);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      Serial.print("\t HIG");
    }
    else 
    { 
      digitalWrite(DPIN_DRV_MODE, LOW);
      analogWrite(DPIN_DRV_EN, angle);
      Serial.print("\tLOW ");
    }
    Serial.print("\2map = ");
    Serial.println(angle);
    analogWrite(DPIN_DRV_EN, angle);
  break; 
  case MODE_BLINKING:
    if (time-lastTime < 250) break;
    lastTime = time;
    blink = !blink;
    digitalWrite(DPIN_DRV_EN, blink);
  break;
  case MODE_DAZZLING:
    if (time-lastTime < 10) break;
    lastTime = time;
    digitalWrite(DPIN_DRV_EN, random(4)<1);
  break;
  case MODE_ABLINKING_PREVIEW:
  case MODE_AUTO_BLINKING:
    //sensor smoothing
    total = total - readings[index];         
    // read from the sensor:  
    readings[index] = readTiltAngle(2); 
    total = total + readings[index];       
    index = index + 1;                    
    if (index >= numReadings) index = 0;                           
    // calculate the average:
    average = total / numReadings;
    // further smoothing with interagtion from last reading and low pass filter  
    smoothedVal = (average * (1 - 0.5)) + (smoothedVal  *  0.5);
    angle = smoothedVal;
    Serial.print("angle = ");
    Serial.print(angle);
    angle = map(angle, 5, 170, 20, 300);
    Serial.print("map = ");
    Serial.println(angle);
    blinkRate=angle;
  case MODE_AUTO_BLINKING_SET:
    if (time-lastTime < blinkRate) break;
    lastTime = time;
    blink = !blink;
    digitalWrite(DPIN_DRV_EN, blink);
  break;  
  }
  
  // Periodically pull down the button's pin, since
  // in certain hardware revisions it can float.
  pinMode(DPIN_RLED_SW, OUTPUT);
  pinMode(DPIN_RLED_SW, INPUT);
  
  // Check for mode changes
  byte newMode = mode;
  byte newBtnDown = digitalRead(DPIN_RLED_SW);
  switch (mode)
  {
  case MODE_OFF:
    if (btnDown && !newBtnDown && (time-btnTime)>20)
      newMode = MODE_AUTO_TILT;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_ABLINKING_PREVIEW;
    break;
  case MODE_AUTO_TILT:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_AUTO_ROLL;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_AUTO_ROLL:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_LOW;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_LOW:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_MED;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_MED:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_HIGH;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_HIGH:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_OFF;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_OFF_PREVIEW:
    // This mode exists just to ignore this button release.
    if (btnDown && !newBtnDown)
      newMode = MODE_OFF;
    break;
  case MODE_ABLINKING_PREVIEW:
    // This mode exists just to ignore this button release.
    if (btnDown && !newBtnDown)
      newMode = MODE_AUTO_BLINKING;
    break;
  case MODE_AUTO_BLINKING:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_AUTO_BLINKING_SET;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_AUTO_BLINKING_SET:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_DAZZLING;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_DAZZLING:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_BLINKING;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  case MODE_BLINKING:
    if (btnDown && !newBtnDown && (time-btnTime)>50)
      newMode = MODE_OFF;
    if (btnDown && newBtnDown && (time-btnTime)>500)
      newMode = MODE_OFF_PREVIEW;
    break;
  }

  // Do the mode transitions
  if (newMode != mode)
  {
    switch (newMode)
    {
    case MODE_OFF:
    case MODE_OFF_PREVIEW:
      Serial.println("Mode = off or off preview");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, LOW);
      digitalWrite(DPIN_DRV_MODE, LOW);
      digitalWrite(DPIN_DRV_EN, LOW);
      break;
    case MODE_AUTO_TILT:
    case MODE_AUTO_ROLL:  
    case MODE_LOW:
      Serial.println("Mode = low or auto");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      analogWrite(DPIN_DRV_EN, 64);
      break;
    case MODE_MED:
      Serial.println("Mode = medium");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, LOW);
      analogWrite(DPIN_DRV_EN, 255);
      break;
    case MODE_HIGH:
      Serial.println("Mode = high");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      analogWrite(DPIN_DRV_EN, 255);
      break;
    case MODE_ABLINKING_PREVIEW:
    case MODE_AUTO_BLINKING:
    case MODE_AUTO_BLINKING_SET:
    case MODE_BLINKING:
    case MODE_DAZZLING:
      Serial.println("Mode = blinking or dazzling or auto blinking");
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);
      digitalWrite(DPIN_DRV_MODE, HIGH);
      break;
    }
    mode = newMode;
  }

  // Remember button state so we can detect transitions
  if (newBtnDown != btnDown)
  {
    btnTime = time;
    btnDown = newBtnDown;
    delay(50);
  }
}

void readAccel(char *acc)
{
  while (1)
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_XOUT);
    Wire.endTransmission(false);       // End, but do not stop!
    Wire.requestFrom(ACC_ADDRESS, 3);  // This one stops.

    for (int i = 0; i < 3; i++)
    {
      if (!Wire.available())
        continue;
      acc[i] = Wire.read();
      if (acc[i] & 0x40)  // Indicates failed read; redo!
        continue;
      if (acc[i] & 0x20)  // Sign-extend
        acc[i] |= 0xC0;
    }
    break;
  }
}

float readTiltAngle(int angle) //Z maybe 1
{
  char acc[3];
  readAccel(acc);
  return acos(acc[angle]/(sqrt(pow(acc[0],2)+pow(acc[1],2)+pow(acc[2],2)))) * 180 / 3.14159265;
}
