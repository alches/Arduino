#include <SPI.h>  // include the SPI library
#include <MS5611.h>

MS5611 ms5611;
float toneFrequency = 0;
float periodFrequency = 0;

// -------------------------------------------------------------------
// Variometer configuration settings
// -------------------------------------------------------------------
// climb threshold = 0.1 m/s (>= beeping climb tone starting at 800Hz, with changing period)
// near thermal threshold = -0.2 (>= beeping near thermal tone, 700Hz with fixed period)
// sink tone threshold = -1.0 (>= no tone)
// sink alarm threshold = -5.0 (>= continuos sink tone starting at 600Hz)
// sink alarm = < sink alarm threshold (continuous tone at 350Hz)
// -------------------------------------------------------------------

int vSpeedUnit = 10;                      // Vertical speed measurement unit
float lastAverageAbsoluteAltitude = 0;

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

void setup() {
    // -------------------------------------------------------------------
    // Beeper initialization
    // -------------------------------------------------------------------
    pinMode(timerVin, OUTPUT);
    pinMode(tonePotenciometerSelect, OUTPUT);
    pinMode(periodPotenciometerSelect, OUTPUT);
    pinMode(periodVin, OUTPUT);
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
    while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
    {
      Serial.println("Could not find a valid MS5611 sensor, check wiring!");
      delay(500);
    }

    // -------------------------------------------------------------------
    // Play welcome sound
    // -------------------------------------------------------------------
    digitalWrite(periodVin, HIGH);
    digitalWrite(timerVin, HIGH);
    setTone(560);delay(30);
    digitalWrite(timerVin, LOW);delay(30);digitalWrite(timerVin, HIGH);
    setTone(832);delay(30);
    digitalWrite(timerVin, LOW);delay(30);digitalWrite(timerVin, HIGH);
    setTone(560);delay(30);
    digitalWrite(timerVin, LOW);delay(30);digitalWrite(timerVin, HIGH);
    setTone(832);delay(30);
    digitalWrite(timerVin, LOW);delay(30);digitalWrite(timerVin, HIGH);
    setTone(560);delay(30);
    digitalWrite(periodVin, LOW);
    digitalWrite(timerVin, LOW);

    // -------------------------------------------------------------------
    // Measure initial average absolute altitude
    // -------------------------------------------------------------------
    int measuringTime = 0;
    lastAverageAbsoluteAltitude = getAverageAbsoluteAltitude(measuringTime);  
}

// the loop function runs over and over again forever
void loop() {
  // measure vSpeed
  float vSpeed = getVerticalSpeed(lastAverageAbsoluteAltitude);
vSpeed=0;
for(float i=0; i<10; i+=0.1)
{
  delay(3000);
  vSpeed = i;
  Serial.println("vSpeed:"+(String)vSpeed);
  // -------------------------------------------------------------------
  // Set appropriate beep
  // -------------------------------------------------------------------  
  // climb threshold = 0.1 m/s (>= beeping climb tone starting at 800Hz, with changing pitch)
  if(vSpeed>=climbThreshold)
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(climbToneBaseFreq+vSpeed*10*climbToneFreqStep);
    setPeriod(climbPeriodBaseFreq+vSpeed*10*climbPeriodFreqStep);
  }
  // near thermal threshold = -0.5 (>= beeping near thermal tone, 700Hz with fixed pitch)
  else if((vSpeed>=nearThermalThreshold)&&(vSpeed<climbThreshold))
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(nearThermalToneFreq);
    setPeriod(nearThermalPeriodFreq);
  }
  // sink tone threshold = -1.0 (>= no tone)
  else if((vSpeed>=sinkThreshold)&&(vSpeed<nearThermalThreshold))
  {
     digitalWrite(timerVin, LOW);
     digitalWrite(periodVin, LOW);
  }
  // sink alarm threshold = -5.0 (>= continuous sink tone starting at 600Hz)
  else if((vSpeed>=sinkAlarmThreshold)&&(vSpeed<sinkThreshold))
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, HIGH);
    setTone(sinkToneBaseFreq+vSpeed*10*sinkToneFreqStep);
    //setPeriod(sinkPeriodFreq);
  }
  // sink alarm = < sink alarm threshold (continuous tone at 350Hz)
  else if(vSpeed<sinkAlarmThreshold)
  {
    digitalWrite(timerVin, HIGH);
    digitalWrite(periodVin, LOW);
    setTone(sinkAlarmToneFreq);
    setPeriod(sinkAlarmPeriodFreq);
  }
}
}

// -------------------------------------------------------------------
// Set resistance for MC41010 digital potentiometer using SPI protocol
// -------------------------------------------------------------------
void setPotentiometerResistance(int slaveSelectPin, byte value) 
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
float resitanceFromFrequency(float frequency, int RA, float capacitance) 
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
  float resistance = resitanceFromFrequency(frequency,periodRA,periodCapacitance);
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
  float altitudesSum = 0;
  for(int i=0; i<5; i++)
  {
    long realPressure = ms5611.readPressure(true);
    altitudesSum += ms5611.getAltitude(realPressure) * 100;
    delay(200);
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
  //Serial.println("averageAbsoluteAltitude:" + (String)averageAbsoluteAltitude + ",lastAverageAbsoluteAltitude:"+(String)lastAverageAbsoluteAltitude+",measuringTime:"+(String)measuringTime);
  vSpeed = round(vSpeed / vSpeedUnit) / 10.0;
  lastAverageAbsoluteAltitude = averageAbsoluteAltitude;
  return vSpeed;
}

