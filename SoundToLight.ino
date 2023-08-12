#include <math.h>
#include <Servo.h>


Servo servoHead;
Servo servoBody;

int LedRed    = 5;
int LedGreen  = 6;
int LedBlue   = 9;
int SoundSensor = A0;

int SoundLevel;
float MaxSoundLevel = 1023;

unsigned long CurrentTime;
//unsigned int LastTime;

int ArrayIndexBpm = 0;
unsigned long LastFrequencySample = 0;
int FrequencySampleRate = 250;
int MaxFrequencySampleTime = 2000;
int BpmSampleCounter;
float Bpm;
float BpmSamples[5];
int LightColorOffset = random(360);
float LightColorAngle;
float BpmMean;
float BpmSum;
bool SamplesEmpty = 1;

unsigned long LastVolumeSample = 0;
int VolumeSampleRate = 5;
int VolumeSampleNumber = 0;
float VolumeGain = 15.0;
float VolumePercent = 0.0;
float VolumeAverage = 0.0;
float VolumeSamples[100];

    int LedColors[3];
unsigned long LastColorUpdate = 0;
int ColorUpdateRate = 0;

// Variables head servo
unsigned long LastPositionUpdateHead = 0;
int PositionUpdateRateHead = 7;
int ActPosHead = 90;
int SetPosHead = ActPosHead;
bool DirectionForwardHead = 1;

// Variables body servo
unsigned long LastPositionUpdateBody = 0;
int PositionUpdateRateBody = 50;
int ActPosBody = 90;
int SetPosBody = ActPosBody;
bool DirectionForwardBody = 1;
bool StopMovement = 1;
 
float unwindValue(float  Variable, float  MaxValue){

  if (Variable > MaxValue)
  {
    Variable = Variable - MaxValue;
  }

  return Variable;
}

int *HsvToRgb(float Hue, float Saturation, float Value, int &RedLed, int &GreenLed, int &BlueLed){

  int BaseInterval = floor(Hue / 60);
  float Interval = (Hue / 60.0 - BaseInterval);
  float p = Value * (1 - Saturation);
  float q = Value * (1 - Saturation * Interval);
  float t = Value * (1 - Saturation * (1 - Interval));
  float RedValue, GreenValue, BlueValue;

  switch (BaseInterval)
  {
    case 0:
      RedValue = Value;
      GreenValue = t;
      BlueValue = p;
      break;

    case 1:
      RedValue = q;
      GreenValue = Value;
      BlueValue = p;
      break;

    case 2:
      RedValue = p;
      GreenValue = Value;
      BlueValue = t;
      break;

    case 3:
      RedValue = p;
      GreenValue = q;
      BlueValue = Value;
      break;

    case 4:
      RedValue = t;
      GreenValue = p;
      BlueValue = Value;
      break;

    case 5:
      RedValue = Value;
      GreenValue = p;
      BlueValue = q;
      break;

    case 6:
      RedValue = Value;
      GreenValue = t;
      BlueValue = p;
      break;

  }

  int ColorCode[3];
  RedLed = RedValue * 255;
  GreenLed = GreenValue * 255;
  BlueLed = BlueValue * 255;

  return ColorCode;

}

void setup() {
  
  Serial.begin(9600);
  pinMode(SoundSensor, INPUT);
  pinMode(LedRed, OUTPUT);
  pinMode(LedGreen, OUTPUT);
  pinMode(LedBlue, OUTPUT);
  
  servoHead.attach(10);
  servoHead.write(ActPosHead);

  servoBody.attach(3);
  servoBody.write(ActPosBody);

  LightColorAngle = LightColorOffset;
}

void loop() {

  CurrentTime = millis();

  SoundLevel = analogRead(SoundSensor);

  // Analyzing BPM
  if (SoundLevel >= MaxSoundLevel)
  { 
    if (CurrentTime > LastFrequencySample + FrequencySampleRate)
    {
      StopMovement = 0;
      SamplesEmpty = 0;
      float PeriodeLength = (CurrentTime - LastFrequencySample);
      float Frequency = 1000.0 / PeriodeLength;
      Bpm = Frequency * 60;
      
      // Limit BPM to 200
      if (Bpm > 210)
      {
        Bpm = 210;
      }

      // Save BPM sample for mean
      BpmSamples[ArrayIndexBpm] = Bpm;

      ArrayIndexBpm++;
      if (ArrayIndexBpm > 4)
      {
        ArrayIndexBpm = 0;
      }

      // Calculate BPM mean
      BpmSampleCounter = 0;
      BpmSum = 0.0;
      for ( int i = 0;
            i < 5;
            i++)
      {
        if (BpmSamples[i] > 0)
        {
          BpmSum += BpmSamples[i];
          BpmSampleCounter ++;
        }
      }

      BpmMean = BpmSum / BpmSampleCounter;
      Serial.println(BpmMean);
      
      LightColorAngle = BpmMean / 210.0;
      LightColorAngle *= 360.0 + LightColorOffset;
      LightColorAngle = unwindValue(LightColorAngle, 360.0);

      // Update movement speed
      float BpmPercentage = BpmMean / 180.0;
      float BpmScale = 1 - BpmPercentage;
      
      PositionUpdateRateBody = BpmScale * 50;
      PositionUpdateRateHead = BpmScale * 50;

      if (PositionUpdateRateBody < 7)
      {
        PositionUpdateRateBody = 7;
        PositionUpdateRateHead = 7;
      }

      LastFrequencySample = CurrentTime;
    }
  }

  // Reset BPM if sample time exceeded
  if (CurrentTime > LastFrequencySample + MaxFrequencySampleTime
      && !SamplesEmpty)
  {
    SamplesEmpty = 1;
    LastFrequencySample = CurrentTime;
    for ( int i = 0;
            i < 5;
            i++)
      {
        BpmSamples[ArrayIndexBpm] = 0;
      }
      LightColorOffset = random(360);
      LightColorAngle = LightColorOffset;
      StopMovement = 1;
  }


  if (CurrentTime > LastVolumeSample + VolumeSampleRate)
  {
    VolumePercent = SoundLevel / MaxSoundLevel - 0.65;
    LastVolumeSample = CurrentTime;
    Serial.print(VolumePercent);
    Serial.print(": ");
    if (VolumePercent < 0)
    {
      VolumePercent = 0;
    }
    Serial.println(VolumePercent);
    
    VolumeSamples[VolumeSampleNumber] = VolumePercent;
    VolumeAverage = 0.0;
    for ( int i = 0;
          i < 100;
          i ++)
    {
      VolumeAverage += VolumeSamples[i];
    }
    VolumeAverage /= 100;
    VolumeAverage *= VolumeGain;
    if (VolumeAverage > 1.0)
    {
      VolumeAverage = 1.0;
    }

    VolumeSampleNumber ++;
    if(VolumeSampleNumber == 100)
    {
      VolumeSampleNumber = 0;
    }

  }


  if (CurrentTime > LastColorUpdate + ColorUpdateRate){
    HsvToRgb(LightColorAngle, 1, VolumeAverage, LedColors[0], LedColors[1], LedColors[2]);

    analogWrite(LedRed, LedColors[0]);
    analogWrite(LedGreen, LedColors[1]);
    analogWrite(LedBlue, LedColors[2]);

    LastColorUpdate = CurrentTime;
  }

  if (ActPosHead == SetPosHead)
  {
    while (abs(ActPosHead - SetPosHead) < 40)
    {
      SetPosHead = random(0, 180);
    }
  }

  if (CurrentTime > LastPositionUpdateHead + PositionUpdateRateHead)
  {
    if (!StopMovement)
    {
      if (ActPosHead > SetPosHead)
      {
        ActPosHead --;
      }
      else if (ActPosHead < SetPosHead)
      {
        ActPosHead ++;
      }
      servoHead.write(ActPosHead);
    }
    LastPositionUpdateHead = CurrentTime;
  }

  if (ActPosBody == SetPosBody)
  {
    while (abs(ActPosBody - SetPosBody) < 40)
    {
      SetPosBody = random(0, 180);
    }
  }

  if (CurrentTime > LastPositionUpdateBody + PositionUpdateRateBody)
  {
    if (!StopMovement)
    {
      if (ActPosBody > SetPosBody)
      {
        ActPosBody --;
      }
      else if (ActPosBody < SetPosBody)
      {
        ActPosBody ++;
      }
      servoBody.write(ActPosBody);
    }
    LastPositionUpdateBody = CurrentTime;
  }
}

