
#ifndef SIMULATION_V1888_HPP_
#define SIMULATION_V1888_HPP_

class Simulation {
private:
  double CarHeight, CarLength, cv, g, h, Lb, mA, mB, mQ, mR, Wb, WR;
  double qc, qe, x, qcDt, qeDt, xDt;
  double FfxCF, FfxEF, FrxAB, FrxAC, FrxAE, FryAB, FryAC, FryAE, FryCF, FryEF, Tw, qcDDt, qeDDt, xDDt, qb, qbDt, qbDDt, Amp, Freq, Tt;
  double Pi = 3.141592653589793, _COEF[14][14], *COEF[14], RHS[14], varArrayToIntegrate[myNumberOfODES], Output[2];
  double DEGtoRAD = 0.0174532925199432957692369;
  unsigned long myNumberOfCallsToMGeqns = 0ul;
  
public:
  void simulate();
};

#endif