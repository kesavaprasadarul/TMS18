 #include "Filters.h"


/* LPF Section */
float LPFilter::ComputeFast(float input)
{
  float RawData = input;
  SmoothDataFP = (SmoothDataFP<< BetaINT)-SmoothDataFP; 
  SmoothDataFP += RawData;
  SmoothDataFP >>= BetaINT;
  return SmoothDataFP;
}

float LPFilter::ComputeFP(float input)
{
  float RawData = input;
  SmoothDataFP = (SmoothDataFP - (BetaFP * (SmoothDataFP - RawData)));
  return SmoothDataFP;
} 


/* PT1 Section */
PT1Filter::PT1Filter(){
  Tau = 60.0F;
}

bool PT1Filter::Initialize(float tau){
  FiltVal = 0;
  Tau = tau;
}

float PT1Filter::Compute(float X, float T){
  static float LastRun = 0;
  float Period = (T <= 0) ? 0.0001 : T; // Float: Division by Zero Prevention
  float FF = Tau / Period;
  float newValue = X;

  /* 
    
    Simply trusted that T is always the sampling rate of the loop function, 
    if not uncomment below code-line. But this is advised not to do, since 
    PT1 cannot handle blank time spaces (X-axis) across a function and will 
    create weird-ass peaks in the waveform. So think well and handle Y-Axis
    values before doing this!

  */

  //  if(millis()-LastRun < Period) Period = millis() - LastRun

  if(millis()-LastRun < Period)
  {
     FiltVal = newValue; 
     return newValue;
  }
  if(LastRun == 0) FiltVal = newValue;
  FiltVal = ((FiltVal * FF) + newValue) / ((FF+1) == 0 ? 0.0001 : FF+1);
  LastRun = millis();
  return FiltVal;
}
