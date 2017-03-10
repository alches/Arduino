#include <SPI.h>  // include the SPI library
#include <Wire.h>
#include <math.h>
#include <MS5611.h>

MS5611 ms5611;
float toneFrequency = 0;
float periodFrequency = 0;
double referencePressure;

// -------------------------------------------------------------------
// Variometer configuration settings
// -------------------------------------------------------------------
// climb threshold = 0.1 m/s (>= beeping climb tone starting at 800Hz, with changing pitch)
// near thermal threshold = -0.5 (>= beeping near thermal tone, 700Hz with fixed pitch)
// sink tone threshold = -1.0 (>= no tone)
// sink alarm threshold = -5.0 (>= continuous sink tone starting at 600Hz)
// sink alarm = < sink alarm threshold (continuous tone at 350Hz)
// -------------------------------------------------------------------

int vSpeedUnit = 10;                      // Vertical speed measurement unit
float vSpeedMeasurementPeriod = 1;        // Vertical speed measurement period in seconds
float lastAverageAbsoluteAltitude = 0;

// Climb variables
float climbThreshold = 10;                // Climb threshold in santimeter/seconds
float climbToneBaseFreq = 800;            // Climb tone base frequency in herz
float climbToneFreqStep = 10;             // Climb tone frequency change step in herz
float climbPeriodBaseFreq = 1;            // Climb period base frequency in herz
float climbPeriodFreqStep = 0.03;         // Climb period frequency change step in herz

// Near thermal variables
float nearThermalThreshold = -50;         // Near thermal threshold in santimeter/seconds
float nearThermalToneFreq = 700;          // Near thermal interval frequency in herz

// Sink threshold
float sinkThreshold = -100;               // Sink threshold in santimeter/seconds
float sinkToneBaseFreq = 600;             // Sink tone base frequency in herz
float sinkToneFreqStep = 10;              // Sink tone frequency change step in herz

// Sink alarm threshold
float sinkAlarmThreshold = -500;          // Sink alarm threshold in santimeter/seconds
float sinkAlarmToneFreq = 350;            // Sink alarm tone frequency in herz

// -------------------------------------------------------------------
// Hardware settings
// -------------------------------------------------------------------
int timerVin = 2;                         // V input for timer
int tonePotenciometerSelect = 3;          // Tone potentiometer select pin
int periodPotenciometerSelect = 4;        // Period potentiometer select pin
int toneRA = 1000;                        // RA setting for tone(ohm)
float toneCapacitance = 0.23;             // Tone capacitor capacitance in mF
int periodRA = 1000;                      // RA setting for period(ohm)
float periodCapacitance = 100;            // Period capacitor capacitance in mF

void setup() {
    // -------------------------------------------------------------------
    // Beeper initialization
    // -------------------------------------------------------------------
    pinMode(timerVin, OUTPUT);
    pinMode(tonePotenciometerSelect, OUTPUT);
    pinMode(periodPotenciometerSelect, OUTPUT);
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
    while(!ms5611.begin(MS5611_STANDARD))
    {
      Serial.println("Could not find a valid MS5611 sensor, check wiring!");
      delay(500);
    }

    // -------------------------------------------------------------------
    // Play welcome sound
    // -------------------------------------------------------------------
    setPeriod(10);
    for(int i=1; i<=8;i++)
    {
      setTone(220*i);
    }
    digitalWrite(timerVin, HIGH);

    // -------------------------------------------------------------------
    // Measure initial average absolute altitude
    // -------------------------------------------------------------------
    int measuringTime = 0;
    lastAverageAbsoluteAltitude = getAverageAbsoluteAltitude(measuringTime);
}

// the loop function runs over and over again forever
void loop() {
  toneFrequency = 298;
  periodFrequency = 0.686;
  setTone(toneFrequency);
  setPeriod(periodFrequency);
  
  digitalWrite(timerVin, HIGH);
  delay(1000000000);
/*
   // Read raw values
  uint32_t rawTemp = ms5611.readRawTemperature();
  uint32_t rawPressure = ms5611.readRawPressure();

  // Read true temperature & Pressure
  double realTemperature = ms5611.readTemperature();
  long realPressure = ms5611.readPressure();

 // Read true temperature & Pressure (with compensation)
  double realTemperature2 = ms5611.readTemperature(true);
  long realPressure2 = ms5611.readPressure(true);
  double realAltitude2 = ms5611.getAltitude(realPressure2);
  
  // Calculate altitude
  float absoluteAltitude = ms5611.getAltitude(realPressure);
  float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);

  Serial.println("--");

  Serial.print(" rawTemp = ");
  Serial.print(rawTemp);
  Serial.print(", realTemp = ");
  Serial.print(realTemperature);
  Serial.println(" *C");

  Serial.print(" rawPressure = ");
  Serial.print(rawPressure);
  Serial.print(", realPressure = ");
  Serial.print(realPressure);
  Serial.println(" Pa");

  Serial.print(" absoluteAltitude = ");
  Serial.print(absoluteAltitude);
  Serial.print(" m, relativeAltitude = ");
  Serial.print(relativeAltitude);    
  Serial.println(" m");

  delay(100);
*/


}

// -------------------------------------------------------------------
// Set resistance for MC41010 digital potentiometer using SPI protocol
// -------------------------------------------------------------------
void setPotentiometerResistance(int & slaveSelectPin, byte value) 
{
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin,LOW);
  SPI.transfer(B00010001); // The command byte
  SPI.transfer(value);     // The data byte
  // take the SS pin high to de-select the chip
  digitalWrite(slaveSelectPin,HIGH); 
}

// -------------------------------------------------------------------
// Calculate potentiometer resistance(ohms) based on needed 
// frequency - (Hz), RA - (ohm), capacitance - (mF) 
// -------------------------------------------------------------------
float resitanceFromFrequency(float & frequency, int & RA, float & capacitance) 
{
  // F=1.44/((RA+RB*2)*C)
  float RB = 0.72/((frequency*capacitance)/1000000) - (float)RA/2.00;
  return RB;
}

// -------------------------------------------------------------------
// Set potentiometer resistance for tone based on desired frequency
// -------------------------------------------------------------------
void setTone(float frequency) 
{
  float resistance = resitanceFromFrequency(frequency,toneRA,toneCapacitance);
  resistance = round(resistance/39.0625);
  if(resistance>255)
    resistance = 255;
  setPotentiometerResistance(tonePotenciometerSelect, resistance);  
}

// -------------------------------------------------------------------
// Set potentiometer resistance for period based on desired frequency
// -------------------------------------------------------------------
void setPeriod(float frequency) 
{
  int resistance = resitanceFromFrequency(frequency,periodRA,periodCapacitance);
  resistance = round(resistance/39.0625);
  if(resistance>255)
    resistance = 255;
  setPotentiometerResistance(periodPotenciometerSelect, resistance);  
}

// -------------------------------------------------------------------
// Calculate current average altitude
// -------------------------------------------------------------------
float getAverageAbsoluteAltitude(int & measuringTime) 
{
  long startMeasuringTime = millis();
  double altitudesSum = 0;
  for(int i=0; i<10; i++)
  {
    double realTemperature = ms5611.readTemperature(true);
    long realPressure = ms5611.readPressure(true);
    altitudesSum += ms5611.getAltitude(realPressure) * 100;
    delay(10);
  }
  measuringTime = millis() - startMeasuringTime;
  return altitudesSum/10;
}

// -------------------------------------------------------------------
// Calculate vertical speed
// -------------------------------------------------------------------
float getVerticalSpeed(float & lastAverageAbsoluteAltitude) 
{
  int measuringTime = 0;
  float averageAbsoluteAltitude = getAverageAbsoluteAltitude(measuringTime);
  float vSpeed = (averageAbsoluteAltitude - lastAverageAbsoluteAltitude) * 1000 / measuringTime;
  vSpeed = round(vSpeed / vSpeedUnit * 10) / 10.0;
  lastAverageAbsoluteAltitude = averageAbsoluteAltitude;
  return vSpeed;
}

