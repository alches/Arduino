#include <Timer.h>
#include <SPI.h>
//#include "PDQ_GFX.h"
//#include "PDQ_ILI9341_config.h"
//#include "PDQ_ILI9341.h"
#include <Adafruit_ILI9341.h>
#include <Adafruit_GFX.h>
//#include <gfxfont.h>
#include <MS5611.h>


// -------------------------------------------------------------------
// Variometer configuration settings
// -------------------------------------------------------------------
// climb threshold = 0.1 m/s (>= beeping climb tone starting at 800Hz, with changing period)
// near thermal threshold = -0.2 (>= beeping near thermal tone, 700Hz with fixed period)
// sink tone threshold = -1.0 (>= no tone)
// sink alarm threshold = -5.0 (>= continuos sink tone starting at 600Hz)
// sink alarm = < sink alarm threshold (continuous tone at 350Hz)
// -------------------------------------------------------------------


// Climb variables
float climbThreshold = 0.1;               // Climb threshold in meter/seconds
float climbToneBaseFreq = 800;            // Climb tone base frequency in herz
float climbToneFreqStep = 10;             // Climb tone frequency change step in herz
float climbPeriodBaseFreq = 1;            // Climb period base frequency in herz
float climbPeriodFreqStep = 0.05;         // Climb period frequency change step in herz

// Near thermal variables
float nearThermalThreshold = -0.2;        // Near thermal threshold in meter/seconds
float nearThermalToneFreq = 700;          // Near thermal interval tone frequency in herz
float nearThermalPeriodFreq = 0.65;       // Near thermal interval period frequency in herz

// Sink threshold
float sinkThreshold = -1.0;               // Sink threshold in meter/seconds
float sinkToneBaseFreq = 600;             // Sink tone base frequency in herz
float sinkToneFreqStep = 10;              // Sink tone frequency change step in herz
float sinkPeriodFreq = 0.65;

// Sink alarm threshold
float sinkAlarmThreshold = -5.0;          // Sink alarm threshold in santimeter/seconds
float sinkAlarmToneFreq = 350;            // Sink alarm tone frequency in herz
float sinkAlarmPeriodFreq = 5;            // Sink alarm tone frequency in herz

// vSpeed calculation variables
float  vSpeedMeasurePeriod = 300;       // In milliseconds

// -------------------------------------------------------------------
// Kalman filter initial variables
// -------------------------------------------------------------------
float varianceMeasurment = 0.15;   // variance determined using excel and reading samples of raw sensor data (sashualo gadaxra)
float varianceProcess = 0.0005;      // larger value - more rapid change but more noise, less velua - slow change according to process measurments but less noise
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

// -------------------------------------------------------------------
// Hardware settings
// -------------------------------------------------------------------
int timerVin = 2;                         // V input for timer
int tonePotenciometerSelect = 3;          // Tone potentiometer select pin
int periodPotenciometerSelect = 4;        // Period potentiometer select pin
int periodVin = 7;                        // Period potentiometer output pin (for switching period timer on/off)
int toneRA = 1000;                        // RA setting for tone(ohm)
float toneCapacitance = 0.23;             // Tone capacitor capacitance in mF
int periodRA = 1000;                      // RA setting for period(ohm)
float periodCapacitance = 100;            // Period capacitor capacitance in mF
int screenSelect = 5;
int screenDataCommand = 6;

// Technical variables
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  
Adafruit_ILI9341 tft = Adafruit_ILI9341(screenSelect, screenDataCommand);
//PDQ_ILI9341 tft;      // PDQ: create LCD object (using pins in "PDQ_ST7735_config.h")
MS5611 ms5611;
float toneFrequency = 0;
float periodFrequency = 0;

int   numberOfAltitudeSums = 0;
float averageAltitudesSum = 0;
long  startTimeMeasure = 0;
float lastAverageAbsoluteAltitude = 0;
float absoluteAltitude = 0;
bool  vSpeedMeasurementInProgress = false;
char  formatedTime[8];
Timer clockTimer;
Timer vSpeedTimer;

void setup() {
  // -------------------------------------------------------------------
  // Beeper initialization
  // -------------------------------------------------------------------
  pinMode(timerVin, OUTPUT);
  pinMode(tonePotenciometerSelect, OUTPUT);
  pinMode(periodPotenciometerSelect, OUTPUT);
  pinMode(periodVin, OUTPUT);
  pinMode(screenSelect, OUTPUT);
  pinMode(screenDataCommand, OUTPUT);
  SPI.begin();                        // Initialize SPI:

  // -------------------------------------------------------------------
  // Pressure initialization
  // -------------------------------------------------------------------
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  // -------------------------------------------------------------------
  Serial.begin(9600);
  while (!ms5611.begin(MS5611_ULTRA_HIGH_RES))
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  // -------------------------------------------------------------------
  // Screen initialization
  // -------------------------------------------------------------------
  tft.begin();
  tft.fillScreen(ILI9341_WHITE);
  //tft.setRotation(1);

  // -------------------------------------------------------------------
  // Play welcome sound
  // -------------------------------------------------------------------
  digitalWrite(periodVin, HIGH);
  digitalWrite(timerVin, HIGH);
  setTone(560); delay(50);
  setTone(832); delay(50);
  setTone(560); delay(50);
  setTone(832); delay(50);
  setTone(560); delay(50);
  digitalWrite(periodVin, LOW);
  digitalWrite(timerVin, LOW);

  // -------------------------------------------------------------------
  // Measure initial average absolute altitude
  // -------------------------------------------------------------------
  startTimeMeasure = millis();
  lastAverageAbsoluteAltitude = getAbsoluteAltitude();

  // -------------------------------------------------------------------
  // Register timer events
  // -------------------------------------------------------------------
  clockTimer.every(1000, drawClock);
  vSpeedTimer.every(10, measureVSpeed);

  // -------------------------------------------------------------------
  // Initialize static graphics
  // -------------------------------------------------------------------
  drawStaticGraphics();
}

// the loop function runs over and over again forever
void loop() {
  // -------------------------------------------------------------------
  // Tick timers
  // -------------------------------------------------------------------  
  clockTimer.update();
  //vSpeedTimer.update();

  //for (int i=1;i<50;i++)
  {
     updateDisplayBar(10.0, 0.0);
     updateDisplayBar(-10.0, 10.0);
     updateDisplayBar(0.0, -10.0);
  }
  
  
}

// -------------------------------------------------------------------
// Set resistance for MC41010 digital potentiometer using SPI protocol
// -------------------------------------------------------------------
void setPotentiometerResistance(int slaveSelectPin, byte value)
{
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(B00010001); // The command byte
  SPI.transfer(value);     // The data byte
  // take the SS pin high to de-select the chip
  digitalWrite(slaveSelectPin, HIGH);
}

// -------------------------------------------------------------------
// Calculate potentiometer resistance(ohms) based on needed
// frequency - (Hz), RA - (ohm), capacitance - (mF)
// -------------------------------------------------------------------
float resitanceFromFrequency(float frequency, int RA, float capacitance)
{
  // F=1.44/((RA+RB*2)*C)
  float RB = 0.72 / ((frequency * capacitance) / 1000000) - (float)RA / 2.00;
  return RB;
}

// -------------------------------------------------------------------
// Set potentiometer resistance for tone based on desired frequency
// -------------------------------------------------------------------
void setTone(float frequency)
{
  float resistance = resitanceFromFrequency(frequency, toneRA, toneCapacitance);
  resistance = round(resistance / 39.0625);
  if (resistance > 255)
    resistance = 255;
  setPotentiometerResistance(tonePotenciometerSelect, resistance);
}

// -------------------------------------------------------------------
// Set potentiometer resistance for period based on desired frequency
// -------------------------------------------------------------------
void setPeriod(float frequency)
{
  float resistance = resitanceFromFrequency(frequency, periodRA, periodCapacitance);
  resistance = round(resistance / 39.0625);
  if (resistance > 255)
    resistance = 255;
  setPotentiometerResistance(periodPotenciometerSelect, resistance);
}

// -------------------------------------------------------------------
// Calculate current absolute altitude
// -------------------------------------------------------------------
float getAbsoluteAltitude()
{
  return ms5611.getAltitude(ms5611.readPressure(true));
}


// -------------------------------------------------------------------
// Set appropriate beep
// -------------------------------------------------------------------
void setBeep(float verticalSpeed)
{
  // climb threshold = 0.1 m/s (>= beeping climb tone starting at 800Hz, with changing pitch)
  if (verticalSpeed >= climbThreshold)
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(climbToneBaseFreq + verticalSpeed * 10 * climbToneFreqStep);
    setPeriod(climbPeriodBaseFreq + verticalSpeed * 10 * climbPeriodFreqStep);
  }
  // near thermal threshold = -0.5 (>= beeping near thermal tone, 700Hz with fixed pitch)
  else if ((verticalSpeed >= nearThermalThreshold) && (verticalSpeed < climbThreshold))
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(nearThermalToneFreq);
    setPeriod(nearThermalPeriodFreq);
  }
  // sink tone threshold = -1.0 (>= no tone)
  else if ((verticalSpeed >= sinkThreshold) && (verticalSpeed < nearThermalThreshold))
  {
    digitalWrite(timerVin, LOW);
    digitalWrite(periodVin, LOW);
  }
  // sink alarm threshold = -5.0 (>= continuous sink tone starting at 600Hz)
  else if ((verticalSpeed >= sinkAlarmThreshold) && (verticalSpeed < sinkThreshold))
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, HIGH);
    setTone(sinkToneBaseFreq + verticalSpeed * 10 * sinkToneFreqStep);
    //setPeriod(sinkPeriodFreq);
  }
  // sink alarm = < sink alarm threshold (continuous tone at 350Hz)
  else if (verticalSpeed < sinkAlarmThreshold)
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(sinkAlarmToneFreq);
    setPeriod(sinkAlarmPeriodFreq);
  }
}

// -------------------------------------------------------------------
// Set appropriate beep
// -------------------------------------------------------------------
void drawClock()
{
  long val = millis()/1000;
  int days = elapsedDays(val);
  int hours = numberOfHours(val);
  int minutes = numberOfMinutes(val);
  int seconds = numberOfSeconds(val);
  
  sprintf(formatedTime,"%02d:%02d:%02d", hours, minutes, seconds);

  //-----------------------------------
  // Update Display timer
  //-----------------------------------
  updateDisplayTimer(formatedTime);
}

// -------------------------------------------------------------------
// Measure vSpeed
// -------------------------------------------------------------------
void measureVSpeed()
{  
  if(!vSpeedMeasurementInProgress)
  {
    vSpeedMeasurementInProgress = true;
    
    // Set all pressure sensor data variables after measurment
    measurePressureSensordata();

    // Calculate vertical speed
    averageAltitudesSum += filterKalman(absoluteAltitude);
    numberOfAltitudeSums++;
    if(millis()-startTimeMeasure >= vSpeedMeasurePeriod || numberOfAltitudeSums >= 100)
    {
      float averageAbsoluteAltitude = averageAltitudesSum / numberOfAltitudeSums;
      float vSpeed = (averageAbsoluteAltitude - lastAverageAbsoluteAltitude) * 1000 / (millis()-startTimeMeasure); //Unit - Meter/Second
      lastAverageAbsoluteAltitude = averageAbsoluteAltitude;
      startTimeMeasure = millis();
      averageAltitudesSum = 0;
      numberOfAltitudeSums = 0;
      vSpeed = round(vSpeed * 10.0) / 10.0; //rounding to 1 decimal digit

      // vSpeed limit control
      if(vSpeed>10)
        vSpeed = 10;
      if(vSpeed<-10)
        vSpeed = -10;

      if(startTimeMeasure > 3000)
      {
        //-----------------------------------
        // Set Beep
        //-----------------------------------
        setBeep(vSpeed);
        
        //-----------------------------------
        // Update Bar
        //-----------------------------------
        //updateDisplayBar(vSpeed);
  
        //-----------------------------------
        // Update Display vSpeed
        //-----------------------------------
        updateDisplayVSpeed(vSpeed);
      }
    }
    vSpeedMeasurementInProgress = false;  
  }
}

// -------------------------------------------------------------------
// Update display vSpeed
// -------------------------------------------------------------------
void updateDisplayBar(float verticalSpeed, float lastVerticalSpeed)
{
  if(verticalSpeed - lastVerticalSpeed == 0)
    return;
      
  uint16_t rectColor = 0;
  int startY = 0;
  int endY = 0;
  int drawDirection = 0;
  int scrW = 240;
  int scrH = 320;
  int border = 5;
  int zeroX = border+34;
  int zeroY = scrH / 2;
  int j=0;
  
  // Up
  if(verticalSpeed - lastVerticalSpeed > 0)
  {
    startY = lastVerticalSpeed * 10;
    endY = verticalSpeed * 10;
    drawDirection = -1;
    for(int i = startY; i<=endY; i++)
    {
      if(i<=50 and i>=-50)
      {
        if(i>0)
          rectColor = ILI9341_BLACK;
        else 
          rectColor = ILI9341_WHITE;
        tft.drawRect(zeroX,zeroY + drawDirection*i*3-2, 50, 2, rectColor);
      }
      else
      {
        if(i>0) 
          j = i - 50;
        else    
          j = i + 50;
        if(j>0)
          rectColor = ILI9341_WHITE;
        else 
          rectColor = 0xB000;
        tft.drawRect(zeroX,zeroY + drawDirection*j*3-2, 50, 2, rectColor);
      }
      delay(10);
    }
  }
  // Down
  else
  {
    startY = lastVerticalSpeed * 10;
    endY = verticalSpeed * 10;
    drawDirection = 1;
    for(int i = startY; i>=endY; i--)
    {
      if(i<=50 and i>=-50)
      {
        if(i<0)
          rectColor = 0xB000;
        else 
          rectColor = ILI9341_WHITE;
        tft.drawRect(zeroX,zeroY - drawDirection*i*3-2, 50, 2, rectColor);
      }
      else
      {
        if(i>0) 
          j = i - 50;
        else    
          j = i + 50;
        if(j<0)
          rectColor = ILI9341_WHITE;
        else 
          rectColor = ILI9341_BLACK;
        tft.drawRect(zeroX,zeroY - drawDirection*j*3-2, 50, 2, rectColor);
      }
      delay(10);
    }
  }
 
}

// -------------------------------------------------------------------
// Draw static objects
// -------------------------------------------------------------------
void drawStaticGraphics()
{
  uint16_t rectColor = 0;
  int scrW = 240;
  int scrH = 320;
  int border = 5;
  int zeroX = border+27;
  int zeroY = scrH / 2;
  int mark = 2;
  rectColor = ILI9341_BLACK;
  for(int i = -50; i<= 50; i++)
  {
    if(i%10==0)
    {
      tft.setCursor(border+13, zeroY + i*3-7);
      tft.setTextColor(0x6D60, ILI9341_WHITE);
      tft.setTextSize(2);
      tft.print(abs(i/10));
      if(i/10 != 0)
      {
        if(abs(i/10)==5)
          tft.setCursor(border, zeroY + i*3-3);
        else
          tft.setCursor(border+6, zeroY + i*3-3);
        tft.setTextColor(0xB000, ILI9341_WHITE);
        tft.setTextSize(1);
        tft.print(abs(i/10)+5);   
      }
      mark = 6;
    }
    else 
      mark = 2;
    tft.drawRect(zeroX,zeroY + i*3-2,mark, 2, rectColor);
  }
}


// -------------------------------------------------------------------
// Update display timer
// -------------------------------------------------------------------
void updateDisplayTimer(char* formatedTime)
{
  tft.setCursor(95, 10);
  tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(formatedTime);
}

// -------------------------------------------------------------------
// Update display vSpeed
// -------------------------------------------------------------------
void updateDisplayVSpeed(float verticalSpeed)
{
  tft.setCursor(60, 150);
  tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
  tft.setTextSize(4);
  if(verticalSpeed>=0) 
    tft.print(' ');
  tft.print(verticalSpeed);
}

// -------------------------------------------------------------------
// Kalman filter
// -------------------------------------------------------------------
float filterKalman(float sensorData)
{
  Pc = P + varianceProcess;
  G = Pc/(Pc + varianceMeasurment);    // kalman gain
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(sensorData-Zp)+Xp;   // the kalman estimate of the sensor data
  return Xe;
}

// -------------------------------------------------------------------
// Measure pressure sensor data
// -------------------------------------------------------------------
void measurePressureSensordata()
{
  absoluteAltitude = getAbsoluteAltitude();
}
