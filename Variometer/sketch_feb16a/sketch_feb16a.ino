#include <Timer.h>
#include <SPI.h>
#include "PDQ_GFX.h"
#include "PDQ_ILI9341_config.h"
#include "PDQ_ILI9341.h"
//#include <Adafruit_ILI9341.h>
//#include <Adafruit_GFX.h>
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

// vSpeed calculation variables
float  vSpeedMeasurePeriod = 1000;       // In milliseconds

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

// -------------------------------------------------------------------
// Hardware settings
// -------------------------------------------------------------------
int timerVin = 2;                         // V input for timer
int tonePotenciometerSelect = 3;          // Tone potentiometer select pin
int periodPotenciometerSelect = 4;        // Period potentiometer select pin
int periodVin = 7;                // Period potentiometer output pin (for switching period timer on/off)
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
//Adafruit_ILI9341 tft = Adafruit_ILI9341(screenSelect, screenDataCommand);
PDQ_ILI9341 tft;      // PDQ: create LCD object (using pins in "PDQ_ST7735_config.h")
MS5611 ms5611;
float toneFrequency = 0;
float periodFrequency = 0;

int   numberOfAltitudeSums = 0;
float averageAltitudesSum = 0;
long  startTimeMeasure = 0;
float vSpeed = 0;
float lastAverageAbsoluteAltitude = 0;
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
  setTone(560); delay(20);
  digitalWrite(timerVin, LOW); delay(20); digitalWrite(timerVin, HIGH);
  setTone(832); delay(20);
  digitalWrite(timerVin, LOW); delay(20); digitalWrite(timerVin, HIGH);
  setTone(560); delay(20);
  digitalWrite(timerVin, LOW); delay(20); digitalWrite(timerVin, HIGH);
  setTone(832); delay(20);
  digitalWrite(timerVin, LOW); delay(20); digitalWrite(timerVin, HIGH);
  setTone(560); delay(20);
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
  vSpeedTimer.every(50, measureVSpeed);
}

// the loop function runs over and over again forever
void loop() {
  // -------------------------------------------------------------------
  // Tick timers
  // -------------------------------------------------------------------  
  clockTimer.update();
  vSpeedTimer.update();  
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
float setBeep(float vSpeed)
{

  // climb threshold = 0.1 m/s (>= beeping climb tone starting at 800Hz, with changing pitch)
  if (vSpeed >= climbThreshold)
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(climbToneBaseFreq + vSpeed * 10 * climbToneFreqStep);
    setPeriod(climbPeriodBaseFreq + vSpeed * 10 * climbPeriodFreqStep);
  }
  // near thermal threshold = -0.5 (>= beeping near thermal tone, 700Hz with fixed pitch)
  else if ((vSpeed >= nearThermalThreshold) && (vSpeed < climbThreshold))
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(nearThermalToneFreq);
    setPeriod(nearThermalPeriodFreq);
  }
  // sink tone threshold = -1.0 (>= no tone)
  else if ((vSpeed >= sinkThreshold) && (vSpeed < nearThermalThreshold))
  {
    digitalWrite(timerVin, LOW);
    digitalWrite(periodVin, LOW);
  }
  // sink alarm threshold = -5.0 (>= continuous sink tone starting at 600Hz)
  else if ((vSpeed >= sinkAlarmThreshold) && (vSpeed < sinkThreshold))
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, HIGH);
    setTone(sinkToneBaseFreq + vSpeed * 10 * sinkToneFreqStep);
    //setPeriod(sinkPeriodFreq);
  }
  // sink alarm = < sink alarm threshold (continuous tone at 350Hz)
  else if (vSpeed < sinkAlarmThreshold)
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
    averageAltitudesSum += getAbsoluteAltitude();
    numberOfAltitudeSums++;
    if(millis()-startTimeMeasure >= vSpeedMeasurePeriod || numberOfAltitudeSums > 100)
    {
      int measuringTime = millis()-startTimeMeasure;
      float averageAbsoluteAltitude = averageAltitudesSum / numberOfAltitudeSums;
      vSpeed = (averageAbsoluteAltitude - lastAverageAbsoluteAltitude) * 1000 / measuringTime; //Unit - Meter/Second
      vSpeed = round(vSpeed * 10.0) / 10.0; //rounding to 1 decimal digit
      lastAverageAbsoluteAltitude = averageAbsoluteAltitude;
      startTimeMeasure = millis();
      averageAltitudesSum = 0;
      numberOfAltitudeSums = 0;

      //-----------------------------------
      // Set Beep
      //-----------------------------------
      //setBeep(vSpeed);

      //-----------------------------------
      // Update Bar
      //-----------------------------------
      //updateDisplayBar(vSpeed);

      //-----------------------------------
      // Update Display vSpeed
      //-----------------------------------
      updateDisplayVSpeed(vSpeed);

    }
    vSpeedMeasurementInProgress = false;  
  }
}

// -------------------------------------------------------------------
// Update display timer
// -------------------------------------------------------------------
void updateDisplayTimer(char* formatedTime)
{
  tft.setCursor(75, 20);
  tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(formatedTime);
}

// -------------------------------------------------------------------
// Update display vSpeed
// -------------------------------------------------------------------
void updateDisplayVSpeed(float vSpeed)
{
  tft.setCursor(60, 150);
  tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
  tft.setTextSize(4);
  if(vSpeed>=0) 
    tft.print(' ');
  tft.print(vSpeed);
}

