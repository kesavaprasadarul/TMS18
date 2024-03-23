#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include "ToF.h"

/* CLASSES */

/* HAPTIC FEEDBACK ACTUATORS */

class DistanceHaptic
{
  private:
    int PinNumber = 14;
    float Distance = 0.0F;
    int PWMValueOld = 0;
    int TimePWMValueOld = 0;
    float Kp = 1;
    float Ki = 1;
    float Kd = 1;
    float AccumulatedError = 0.0F;


  public:
    /* CALIBRATIONS */
    int MaxVibrationIntensity = 255;
    int MinVibrationIntensity = 140;
    int VibrationDistanceThresholdMax = 500; //in mm
    int VibrationDistanceThresholdMin = 40; //in mm
    int HysterisisGainAt = 150;
    bool IsHysterisisCompensationEnabled = false;

    /* FUNCTIONS */
    float GetVibrationIntensityForDistance (float distance, float stabilityFactor, bool isHysterisisCompensationEnabled, bool isStable);

};

float DistanceHaptic::GetCompensationForVibrationChange(int RequestedPWM)
{
  if(RequestedPWM > CurrentPWMValue)
  {
    /* FIND ERROR E(t) */
    float CurrentError = (CurrentPWMValue - RequestedPWM) / (milis() - TimePWMValueOld);
    float AccumulatedError += CurrentError;
    
  }
}

float DistanceHaptic::GetVibrationIntensityForDistance(float distance, float stabilityFactor, bool isHysterisisCompensationEnabled, bool isStable)
{
  if(distance < VibrationDistanceThresholdMax)
  {
    float vibrationValue = ((1-((distance - VibrationDistanceThresholdMin) / (VibrationDistanceThresholdMax - VibrationDistanceThresholdMin))) * (MaxVibrationIntensity - MinVibrationIntensity)) + MinVibrationIntensity;
    if(isStable)
    {
      vibrationValue -= ((1-stabilityFactor) * (MaxVibrationIntensity - MinVibrationIntensity));
    }
    return max(MinVibrationIntensity, min(MaxVibrationIntensity, vibrationValue));
  }
  return 0;
}


DFRobot_VL53L0X sensor;
ToF ToF1(20, 100, 1500, 0, 0, 2500, SAMPLING_TIME);
DistanceHaptic Haptic;

void setup() {
  //initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(14, OUTPUT);
  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous,sensor.eHigh);
  //Laser rangefinder begins to work
  sensor.start();

}

void loop() {
  // put your main code here, to run repeatedly:
  float ToFDistance = sensor.getDistance();
  ToF1.SetDistance(ToFDistance);
  float value = Haptic.GetVibrationIntensityForDistance(ToF1.DistanceFiltered, ToF1.GainDel2Factor, false, ToF1.IsStable);
  analogWrite(14,(int) value);
  print("DistanceRaw(mm):"); print(ToF1.DistanceUnfiltered);
  print(",DistanceFiltered(mm):"); print(ToF1.DistanceFiltered);
  print(",Delta1(cm/s):"); print(ToF1.Del1);
  print(",Delta2(cm/s^2):"); print(ToF1.Del2);
  print(",GainFactor(-):"); print(ToF1.GainDel2Factor);
  print(",IsStable:"); print(ToF1.IsStable);
  print(",IsDampening:"); print(ToF1.IsDampening);  
  print(",VibrationValue:"); print(value);  
  Serial.println("");
  delay(SAMPLING_TIME);
}

