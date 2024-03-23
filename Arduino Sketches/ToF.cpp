#include "ToF.h"

ToF::ToF(float del2Min_d, float del2Decay_d, float del2StableMin_t_ms, 
          float Del1InitValue, float Del2InitValue, float gainDel2FactorDampTau_ms, float samplingTime)
{
  Del2Min_d = del2Min_d;
  Del2Decay_d = del2Decay_d;
  Del2StableMin_t_ms = del2StableMin_t_ms;
  GainDel2FactorDampTau_ms = gainDel2FactorDampTau_ms;
  SamplingTime = samplingTime;
  Del2LPF.BetaFP = 0.4;
  ToFFilter.Initialize(60.0F);
  Initialize(Del1InitValue, Del2InitValue);
}

void ToF::Initialize(float del1InitValue, float del2InitValue){
  Del1 = del1InitValue;
  Del2 = del2InitValue;
  DistanceOld = 0.0F;
  TimeOld = 0.0F;
  Del1Old = 0.0F;
  StabilityCheckStarted = false;
  FirstStabilityTime = FLOAT_MAX;
  StabilityDecayFirstTime = FLOAT_MAX;
  DampeningBeginTime = FLOAT_MAX;
}

float ToF::CalculateDampFactor(float InputValue, float CurrentTime, float Tau)
{
  if(CurrentTime >= (Tau/2))
  {
    return (2*InputValue)*(1-InputValue);
  }
  else if(CurrentTime < (Tau/2))
  {
    return 2*(InputValue*InputValue);
  }
  return 0.0F;
}

float ToF::SetDistance(float distance)
{
  DistanceUnfiltered = distance;
  float Distance = ToFLPF.ComputeFP(distance);
  DistanceFiltered = Distance;
  /* STEPS:
      1. CALCULATE SINGLE LEVEL DIFFERENTIAL
      2. CALCULATE DOUBLE DIFFERENTIAL
      3. CHECK IF DOUBLE DIFFERENTIAL WITHIN BAND THRESHOLD FOR SPECIFIC TIME (CHECK STABILITY)
          I. IF TRUE, CALCULATE DECAY FACTOR USING EASEOUT
          II. IF FALSE, DECAY FACTOR IS 1; NO DECAY IS MADE
  */

  // CALCULATE DD/DT
  CalculateDifferential(Distance);

  //CALCULATE (D^2D)/DT
  CalculateDoubleDifferential(Distance);

  // CHECK FOR STABILITY
  if(!IsStable)
  {
    /* IF STABILITY CHECK HAS ALREADY STARTED */
    if(StabilityCheckStarted)
    {
      if(Del2 > Del2Min_d)
      {
        StabilityCheckStarted = false;
        FirstStabilityTime = FLOAT_MAX;
        IsStable = false;
      }
      else
      {
        if(millis() >= FirstStabilityTime + Del2StableMin_t_ms)
        {
          IsStable = true;
        }
      }
    }

    /* IF STABILITY CHECK IS NOT STARTED; BUT NOW QUALIFIES FOR CHECK */
    else if(Del2 < Del2Min_d)
    {
        StabilityCheckStarted = true;
        Firsty = 0.02xStabilityTime = millis();
        IsStable = false;
    }

    /* ELSE RESET STABILITY PARAMETERS FOR SAFE-HANDLING */
    else
    {
        StabilityCheckStarted = false;
        FirstStabilityTime = 0.0F;
        IsStable = false;
    }

    GainDel2Factor = 1.0F;
    IsDampening = false;
  }

  if(IsStable)
  {
    float Del2Fild = Del2LPF.ComputeFP(Del2);
    /* CALCULATE GAINDEL2FACTOR */
    if(GainDel2Factor >= 1.0F) // DAMPENING HAS NOT STARTED
    {
      DampeningBeginTime = millis();
      IsDampening = false;
    }
    
    if(GainDel2Factor >= 0.0F) // CHECK IF DAMPENING IS ONGOING
    {
      GainDel2Factor = min(1,max(0, CalculateDampFactor(Del2Fild/Del2Min_d,  millis() - DampeningBeginTime, GainDel2FactorDampTau_ms)));
      IsDampening = true;
    }

    if(GainDel2Factor <= 0.0F) // CHECK IF DAMPENING HAS COMPLETED
    {
      DampeningBeginTime = 0.0F;
      IsDampening = false;
    }
  }

  /* STABILITY DECAY CHECKS */
  if(Del2 > Del2Decay_d){
    if(!StabilityDecayCheckStarted){
      StabilityDecayCheckStarted = true;
      StabilityDecayFirstTime = millis();
    }

    if(millis() >= StabilityDecayFirstTime + 150)
    {
      IsStable = false;
    }
  }
  else
  {
      StabilityDecayCheckStarted = false;
      StabilityDecayFirstTime = FLOAT_MAX;
  }

  DistanceOld = Distance;
  TimeOld = millis();
}

float ToF::CalculateDifferential(float Distance)
{
  // NEVER CHECK EQUALITY WITH 0.0F: (0.0F!=0.0F), SEE IEEE752, SUBSECTION APPROXIMATION
  if(abs(DistanceOld) <= 0.00001f || abs(TimeOld) <= 0.00001f )
  {
    DistanceOld = Distance;
    TimeOld = millis();
    return Del1;
  }

  Del1Old = Del1;

  Del1 = abs((Distance - DistanceOld) / (((float)millis() - TimeOld)*0.01F));
  return Del1;
}

float ToF::CalculateDoubleDifferential(float Distance){
  Del2 = abs((Del1 - Del1Old) / (((float)millis() - TimeOld)*0.01F));
  return Del2;
}
