#include "Constants.h"
#include "Filters.h"

class ToF{
  private:
    float DistanceOld;
    float TimeOld;
    float Del1Old;
    bool  StabilityCheckStarted;
    bool  StabilityDecayCheckStarted;
    float FirstStabilityTime;
    float DampeningBeginTime;
    float StabilityDecayFirstTime;
    PT1Filter ToFFilter;
    LPFilter ToFLPF;
    LPFilter Del2LPF;

    /* Internal Functions */
    float CalculateDifferential(float Distance);
    float CalculateDoubleDifferential(float Distance);
    float CalculateDampFactor(float InputValue, float CurrentTime, float Tau);

    /* Calibrations */
    float Del2Min_d; // Threshold double-differential distance before stablity is declared
    float Del2Decay_d; // Decay Threshold until when dt^2 ramp down occurs
    float Del2StableMin_t_ms; // Time Threshold until double-differential must be within threshold to declare stability
    float GainDel2FactorDampTau_ms; // Tau value for damping speed
    float SamplingTime = 0.0F; // Sampling Time for Filtering


  public:
    /* Functions */
    void Initialize(float del1InitValue, float del2InitValue);
    float SetDistance(float Distance);

    /* Constructor */
    ToF(float del2Min_d, float del2Decay_d, float del2StableMin_t_ms, float Del1InitValue, 
        float Del2InitValue, float gainDel2FactorDampTau_ms, float samplingTime);

    /* Messages */
    float DistanceUnfiltered = 0;
    float DistanceFiltered = 0.0F;
    float GainDel2Factor = 0.0F;
    bool IsStable = false;
    bool IsDampening = false;
    float Del1 = 0.0F;
    float Del2 = 0.0F;
};

