 #include "Constants.h"

class LPFilter
{
  private:
    signed long SmoothDataINT;
    signed long SmoothDataFP;
  
  public:
  /* Calibrations */
    int BetaINT = 2; // Typically, this is the length of the LPF (W<16)
    float BetaFP = 0.25; // 0<BetaFP<=1
    float ComputeFast(float input);
    float ComputeFP(float input);
};

class PT1Filter{
  private:
    float FiltVal = 0;
    float Tau = 0;
  public:
    float Compute(float X, float T);
    bool Initialize(float tau);
    PT1Filter();
};