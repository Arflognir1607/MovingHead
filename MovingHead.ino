#include <math.h>
#include <Servo.h>
#define getName(var)  #var

// Debug active
bool Debug = 0;

// Define servo motors
Servo ServoHead;
Servo ServoArm;

// Define PWM output pins
int PinLedRed   = 5;
int PinLedGreen = 6;
int PinLedBlue  = 9;

// Define analog input pins
int PinMicro    = A0;

// Define clock variables
unsigned long Time;
unsigned long BpmSampleTime     = 0;
unsigned long VolumeSampleTime  = 0;
unsigned long ColorChangeTime   = 0;
unsigned long ServoHeadTime     = 0;
unsigned long ServoArmTime      = 0;

// Define sound levels
int SoundCurrent;                 // Current level from microfone
float SoundMax    = 1023;         // Max level of microfone

// Define variables for BPM analyzation
int BpmIndex      = 0;            // Index for BPM sample array
int BpmCounter    = 0;            // Counter for available BPM samples
int BpmSampleRate = 250;          // Max sampling rate [ms]
int BpmMaxSample  = 2000;         // Max time with no BPM sample [ms]

float BpmCurrent;                 // Currently detected BPM
float BpmMax = 210.0;             // Max detectable BPM
float BpmSum;                     // Sum of all BPM samples
float BpmMean;                    // Mean BPM over all samples
float BpmSamples[5];              // Array for BPM samples

bool BpmEmpty     = 1;            // True = BPM sample array is empty

// Define variables for light color control
int ColorOffset   = random(360);  // Initial color angle
int ColorLedValue[3];             // Current values for LED PWM control

float ColorHue;                   // HUE color angle

// Define variables for volume analyzation and light value
int VolumeIndex       = 0;        // Index for colume sample array
int VolumeSampleRate  = 5;        // Max sampling rate [ms]

float VolumeGain          = 15.0; // Gain for volume
float VolumeAverage       = 0.0;  // Average volume value
float VolumePercentageOld = 0.0;  // Last average volume value
float VolumePercent       = 0.0;  // Average volume value [%]
float VolumeSamples[100];         // Array for volume samples

// Define variables for head movement
int HeadActualPos     = 90;       // Actual position of head servo
int HeadSetPos        = 90;       // Set position of head servo

// Define variables for arm movement
int ArmActualPos      = 90;       // Actual position of arm servo
int ArmSetPos         = 90;       // Set position of arm servo

// Define variables for general servo movements
int   ServoSlowRate     = 40;
int   ServoUpdateRate   = 40;     // Update rate for servo set position to control velocity [ms]
bool  ServoStopMovement = 1;      // True = Stop servo movements




void setup() {
  // Set up serial connection
  Serial.begin(9600);

  // Set up PWM pins
  pinMode(PinLedRed,    OUTPUT);
  pinMode(PinLedGreen,  OUTPUT);
  pinMode(PinLedBlue,   OUTPUT);

  // Set up analog input pin
  pinMode(PinMicro,     INPUT);

  // Set up servo motors
  ServoHead.attach(10);
  ServoArm.attach(3);

  ServoHead.write(HeadActualPos);
  ServoArm.write(ArmActualPos);

  // Initialize light color
  ColorHue = ColorOffset;
}

void loop() {
  // Get runtime
  Time = millis();

  // Read inputs
  SoundCurrent = analogRead(PinMicro);

  // Analyzing BPM
  if (SoundCurrent >= SoundMax)
  {
    // Take sample
    if (Time > BpmSampleTime + BpmSampleRate)
    {
      BpmEmpty          = 0;
      // Calculate current BPM
      float Frequency   = 1000.0 / (Time - BpmSampleTime);      // BPM Frequency [1/s]
      BpmCurrent        = Frequency * 60.0;

      // Limit BPM to max value
      if (BpmCurrent > BpmMax)
      {
        BpmCurrent = BpmMax;
      }

      // Add current BPM to samples
      BpmSamples[BpmIndex]  = BpmCurrent;
      BpmIndex++;

      // Reset BPM array index
      if (BpmIndex > 4)
      {
        BpmIndex = 0;
      }

      // Calculate BPM mean value
      BpmSum      = 0;
      BpmCounter  = 0;

      for (int i = 0; i < 5; i++)
      {
        if (BpmSamples[i] != 0)
        {
          BpmSum += BpmSamples[i];
          BpmCounter++;
        }
      }

      BpmMean = BpmSum / BpmCounter;

      // Calculate light color hue
      ColorHue = BpmMean / BpmMax * 360.0 + ColorOffset;
      fc_Unwind(ColorHue, 360.0);

      // Calculate movement speed
      ServoUpdateRate = (1 - BpmMean / 180.0) * ServoSlowRate;

      if (ServoUpdateRate < 7)
      {
        ServoUpdateRate = 7;
      }

      if (Debug)
      {
        // Debug output
        Serial.print(Time);
        Serial.print(" - Current BPM:");
        Serial.print(BpmCurrent);
        Serial.print(" - Mean BPM:");
        Serial.print(BpmMean);
        Serial.print(" - Light hue:");
        Serial.print(ColorHue);
        Serial.print(" - Servo update rate:");
        Serial.println(ServoUpdateRate);
      }

      BpmSampleTime = Time;
    }
  }

  // Reset BPM samples
  if (  Time > BpmSampleTime + BpmMaxSample
      && !BpmEmpty)
  {
    BpmEmpty = 1;
    BpmSampleTime = Time;
    ServoUpdateRate = ServoSlowRate;

    for (int i = 0; i < 5; i++)
    {
      BpmSamples[i] = 0;
    }

    ColorOffset       = random(360);
    ColorHue          = ColorOffset;
    if (Debug)
    {
      // Debug output
      Serial.print(Time);
      Serial.print(" - Resetting BPM samples");
      Serial.print(" - Light hue:");
      Serial.println(ColorHue);
    }
  }

  // Analyze volume and light brightness
  if (Time > VolumeSampleTime + VolumeSampleRate)
  {
    // Calculate volume percentage with filter
    VolumePercent = SoundCurrent / SoundMax - 0.65;

    if (VolumePercent < 0)
    {
      VolumePercent = 0;
    }

    // Add current volume to sample
    VolumeSamples[VolumeIndex]  = VolumePercent;
    VolumeIndex ++;

    if (VolumeIndex > 99)
    {
      VolumeIndex = 0;
    }

    // Calculate gained average volume
    VolumeAverage               = 0.0;

    for (int i = 0; i < 100; i++)
    {
      VolumeAverage += VolumeSamples[i];
    }

    VolumeAverage /= 100.0;
    VolumeAverage *= VolumeGain;

    if (VolumeAverage > 1.0)
    {
      VolumeAverage = 1.0;
    }
    
    if (VolumeAverage > 0.01)
    {
      ServoStopMovement = 0;
    }
    else
    {
      ServoStopMovement = 1;
    }

    // Debug output
    if (VolumePercentageOld != VolumePercent
        && Debug)
    {
      Serial.print(Time);
      Serial.print(" - Current volume percentage:");
      Serial.print(VolumePercent);
      Serial.print(" - Average volume:");
      Serial.println(VolumeAverage);
      VolumePercentageOld = VolumePercent;
    }

    VolumeSampleTime = Time;
  }

  // Update light color
  fc_ConvertHsvToRgb(ColorHue, 1.0, VolumeAverage, ColorLedValue);

  analogWrite(PinLedRed,   ColorLedValue[0]);
  analogWrite(PinLedGreen, ColorLedValue[1]);
  analogWrite(PinLedBlue,  ColorLedValue[2]);

  // Update movements
  fc_ServoMovement(ServoHeadTime, HeadActualPos,  HeadSetPos, ServoHead,  "Head");
  fc_ServoMovement(ServoArmTime,  ArmActualPos,   ArmSetPos,  ServoArm,   "Arm");

}

void fc_ServoMovement(unsigned long &UpdateTime, int &ActPos, int &SetPos, Servo &ServoDrive, char* Name)
{
  // Next update possible
  if (Time > UpdateTime + ServoUpdateRate)
  {
    // Movement is not stopped
    if (!ServoStopMovement)
    {
      // Update movement position negative direction
      if (ActPos < SetPos)
      {
        ActPos++;
      }
      // Update movement position positive direction
      else if (ActPos > SetPos)
      {
        ActPos--;
      }
      // Write position to servo
      ServoDrive.write(ActPos);
    }

    // Update set position
    if (ActPos == SetPos)
    {
      while (abs(ActPos - SetPos) < 40)
      {
        SetPos = random(0, 180);
      }

      if (Debug)
      {
        // Debug output
        Serial.print(Time);
        Serial.print(" - ");
        Serial.print(Name);
        Serial.print(" - next position:");
        Serial.println(SetPos);
      }
      
    }
    UpdateTime = Time;
  }
}

void fc_ConvertHsvToRgb(float Hue, float Saturation, float Value, int (&Led)[3])
// Converting HUE color values to RGB values
{
  // Calculate interime variables
  int   ColorBase     = floor(Hue / 60);
  float ColorInterval = (Hue / 60.0 - ColorBase);
  float p             = Value * (1.0 - Saturation);
  float q             = Value * (1.0 - Saturation * ColorInterval);
  float t             = Value * (1.0 - Saturation * (1.0 - ColorInterval));

  // Calculate RGB colors
  switch (ColorBase)
  {
    case 0:
    case 6:
            Led[0] = Value * 255;
            Led[1] = t * 255;
            Led[2] = p * 255;
            break;
    
    case 1:
            Led[0] = q * 255;
            Led[1] = Value * 255;
            Led[2] = p * 255;
            break;
    
    case 2:
            Led[0] = p * 255;
            Led[1] = Value * 255;
            Led[2] = t * 255;
            break;
    
    case 3:
            Led[0] = p * 255;
            Led[1] = q * 255;
            Led[2] = Value * 255;
            break;
    
    case 4:
            Led[0] = t * 255;
            Led[1] = p * 255;
            Led[2] = Value * 255;
            break;

    case 5:
            Led[0] = Value * 255;
            Led[1] = p * 255;
            Led[2] = q * 255;
            break;
  }
}

void fc_Unwind(float &Input, float MaxValue)
// Unwinding input variable if max value is exceeded
{
  while (Input > MaxValue)
  {
    Input -= MaxValue;
  }
}
