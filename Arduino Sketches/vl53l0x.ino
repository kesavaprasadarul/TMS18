/*!
 * @file vl53l0.ino
 * @brief DFRobot's Laser rangefinder library. The example shows the usage of VL53L0X in a simple way.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [LiXin](xin.li@dfrobot.com)
 * @version  V1.0
 * @date  2017-8-21
 * @url https://github.com/DFRobot/DFRobot_VL53L0X
 */
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

DFRobot_VL53L0X sensor;

bool PT1Messung(float &FiltVal, float Kanal, float Periode, float Tau){
  static float lastRun = 0;
  float FF = Tau / (Periode == 0 ? 0.00001 : Periode);
  float neuWert = Kanal;
  if (millis()-lastRun < Periode) return false;
  if (lastRun == 0) FiltVal = neuWert; 
  FiltVal= ((FiltVal * FF) + neuWert) / ((FF+1) == 0 ? 0.0001 : FF+1); 

  lastRun = millis();
  return true;
}

static float del2 = -16384.0;
static float del1 = -16384.0;
static float valueOld = 0.0f;

static float rawOld = 0.0f;
static float del1Old = 0.0f;

float CalculateStabilityFactor(float ValueIn, float Threshold, float expGain = 4)
{
  float SensorAngleThreshold = 900.0f;

  // if(del2 == -16384.0f) //IMPLAUSIBLE VALUE FOR CURRENT CALCULATION
  // {
  //   if(del1 == -16384.0f)
  //   {
  //     valueOld = ValueIn; 
  //     del1 = 0.0f;
  //     return 0.0f;
  //   }

  //   del2 = del1;
  //   del1 = abs(valueOld - abs(ValueIn));   
  //   valueOld = ValueIn; 
  //   return 0;
  // }

  // del1 = abs(valueOld - abs(ValueIn));   
  // valueOld = ValueIn; 
  // del2 = abs(del2 - del1);
  if(del1Old == 0)
  {
    if(rawOld == 0)
    {
      rawOld = abs(ValueIn);
      return 0;
    }

    del1Old = abs(ValueIn - rawOld);
    return 0;
  }

  del1 = abs(ValueIn - rawOld);
  rawOld = abs(ValueIn);
  del2 = abs(del1 - del1Old);
  del1Old = abs(del1);

  if(ValueIn > SensorAngleThreshold) return 0;
  //Serial.print("Gain Del1: "); Serial.print(del1);Serial.print("\t"); Serial.print("Gain Del2: "); Serial.println(del2);
  return exp((-del2)/expGain);

}


void setup() {
  //initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
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

/*

 */

void loop() 
{
  //Get the distance
  static float FiltVal;
  static float FiltDelVal;
  float SamplingPeriod_ms = 120;

  float DistanceThresMin = 500; //cm
  float DistanceThresMax = 40; //cm
  int VibrationValueMin = 160;
  int VibrationValueMax = 255;

  float ToFDistance = sensor.getDistance();
  
  float ToFFild = PT1Messung(FiltVal, ToFDistance, SamplingPeriod_ms, 60);
  float StabilityFactor = CalculateStabilityFactor(FiltVal, 250.0f, 4);

  int FiltValInt = (int)floor(FiltVal);
  int StabilityValueInt = (int)(StabilityFactor*100.0f);

  int PWMOutVal = 0;
  if(FiltValInt <= DistanceThresMin)
  {
    float VibrationFactor = (FiltValInt - (DistanceThresMin - DistanceThresMax)) / (DistanceThresMin - DistanceThresMax);
    PWMOutVal = VibrationValueMin - (int)(((float)VibrationFactor) * ((float)VibrationValueMax - (float)VibrationValueMin)) ;
  }


  Serial.print(ToFDistance); Serial.print("\t"); Serial.print(StabilityValueInt); Serial.print("\t"); Serial.print(FiltValInt);Serial.print("\t"); 
  Serial.print(PWMOutVal);Serial.print("\t"); Serial.print(StabilityValueInt | (FiltValInt << 16));

  Serial.println("");
  analogWrite(14, PWMOutVal);

  //The delay is added to demonstrate the effect, and if you do not add the delay,
  //it will not affect the measurement accuracy
  //delay(SamplingPeriod_ms);
}