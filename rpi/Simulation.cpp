/* ------------------------------------------------------------------------ */
/* File: Projet.c created Jun 16 2024 by MotionGenesis 5.9.                 */
/* Portions copyright (c) 2009-2019 Motion Genesis LLC.  Rights reserved.   */
/* MotionGenesis Get Started (Home) Licensee: Universite de Sherbrooke. (until August 2024). */
/* Paid-up MotionGenesis Get Started (Home) licensees are granted the right */
/* to distribute this code for legal student-academic (non-professional) purposes only, */
/* provided this copyright notice appears in all copies and distributions.  */
/* ------------------------------------------------------------------------ */
/* The software is provided "as is", without warranty of any kind, express or     */
/* implied, including but not limited to the warranties of merchantability or     */
/* fitness for a particular purpose. In no event shall the authors, contributors, */
/* or copyright holders be liable for any claim, damages or other liability,      */
/* whether in an action of contract, tort, or otherwise, arising from, out of, or */
/* in connection with the software or the use or other dealings in the software.  */
/* ------------------------------------------------------------------------ */
#include "Simulation.hpp"


/* ------------------------------------------------------------------------ */
void  Simulation::Simulate(double tInitial, float height, double Torque[100])
{
   h = height;

   double printIntScreenDbl {0}, printIntFileDbl {0};

   /* Initialize COEF pointers to beginning of each row */
   { int iloop;  for( iloop = 0;  iloop < 14;  iloop++ )  COEF[iloop] = &(_COEF[iloop][0]); }

   /* Unit conversions */
   qc *= 0.0174532925199433;
   qe *= 0.0174532925199433;
   qcDt *= 0.0174532925199433;
   qeDt *= 0.0174532925199433;
   CalculateConstants();
   double tFinal = Tt;
   double tStep = (tFinal-tInitial)/100;

   /* Numerically integrate. */
   MGIntegrateForwardOrBackward( myNumberOfODES, varArrayToIntegrate, Output, tInitial, tFinal, tStep, (int)printIntScreenDbl, (int)printIntFileDbl, Torque );

}

/* ------------------------------------------------------------------------ */
   void  Simulation::CalculateConstants( void )
{
   Freq = sqrt(g/Lb-pow(cv,2));
   Tt = 7.853981633974483/Freq;
   Amp = acos(1-h/Lb);
}


/* ------------------------------------------------------------------------ */
   void  Simulation::CalculateSpecifiedAssignedQuantities( double t, char isIntegratorBoundary )
{
   qb = Amp*t*sin(7.853981633974483*t/Tt)/Tt;
   qbDt = Amp*(Tt*sin(7.853981633974483*t/Tt)+7.853981633974483*t*cos(7.853981633974483*t/Tt))/pow(Tt,2);
   qbDDt = 15.70796326794897*Amp*(cos(7.853981633974483*t/Tt)-3.926990816987241*t*sin(7.853981633974483*t/Tt)/Tt)/pow(Tt,2);
}


/* ------------------------------------------------------------------------ */
   void  Simulation::CalculateCoefficientMatrix( double *coef[], double t, char isIntegratorBoundary )
{
   coef[0][0] = mA;
   coef[0][1] = 0;
   coef[0][2] = 0;
   coef[0][3] = 0;
   coef[0][4] = -1;
   coef[0][5] = 0;
   coef[0][6] = 1;
   coef[0][7] = 0;
   coef[0][8] = 0;
   coef[0][9] = 0;
   coef[0][10] = 1;
   coef[0][11] = 0;
   coef[0][12] = 0;
   coef[0][13] = 0;
   coef[1][0] = 0;
   coef[1][1] = 0;
   coef[1][2] = 0;
   coef[1][3] = 0;
   coef[1][4] = 0;
   coef[1][5] = 1;
   coef[1][6] = 0;
   coef[1][7] = 1;
   coef[1][8] = 0;
   coef[1][9] = 0;
   coef[1][10] = 0;
   coef[1][11] = 1;
   coef[1][12] = 0;
   coef[1][13] = 0;
   coef[2][0] = 0;
   coef[2][1] = 0;
   coef[2][2] = 0;
   coef[2][3] = -1;
   coef[2][4] = -0.5*CarHeight;
   coef[2][5] = 0;
   coef[2][6] = 0.5*CarHeight;
   coef[2][7] = 0.5*CarLength;
   coef[2][8] = 0;
   coef[2][9] = 0;
   coef[2][10] = 0.5*CarHeight;
   coef[2][11] = -0.5*CarLength;
   coef[2][12] = 0;
   coef[2][13] = 0;
   coef[3][0] = mB + mQ;
   coef[3][1] = 0;
   coef[3][2] = 0;
   coef[3][3] = 0;
   coef[3][4] = 1;
   coef[3][5] = 0;
   coef[3][6] = 0;
   coef[3][7] = 0;
   coef[3][8] = 0;
   coef[3][9] = 0;
   coef[3][10] = 0;
   coef[3][11] = 0;
   coef[3][12] = 0;
   coef[3][13] = 0;
   coef[4][0] = 0;
   coef[4][1] = 0;
   coef[4][2] = 0;
   coef[4][3] = 0;
   coef[4][4] = 0;
   coef[4][5] = -1;
   coef[4][6] = 0;
   coef[4][7] = 0;
   coef[4][8] = 0;
   coef[4][9] = 0;
   coef[4][10] = 0;
   coef[4][11] = 0;
   coef[4][12] = 0;
   coef[4][13] = 0;
   coef[5][0] = 0.5*Lb*(2*mQ*(1-mQ/(mB+mQ))+mB*(1-(mB+3*mQ)/(mB+mQ)))*cos(qb);
   coef[5][1] = 0;
   coef[5][2] = 0;
   coef[5][3] = 0;
   coef[5][4] = -0.5*Lb*(mB+2*mQ)*cos(qb)/(mB+mQ);
   coef[5][5] = 0.5*Lb*(mB+2*mQ)*sin(qb)/(mB+mQ);
   coef[5][6] = 0;
   coef[5][7] = 0;
   coef[5][8] = 0;
   coef[5][9] = 0;
   coef[5][10] = 0;
   coef[5][11] = 0;
   coef[5][12] = 0;
   coef[5][13] = 0;
   coef[6][0] = mR;
   coef[6][1] = 0;
   coef[6][2] = 0;
   coef[6][3] = 0;
   coef[6][4] = 0;
   coef[6][5] = 0;
   coef[6][6] = -1;
   coef[6][7] = 0;
   coef[6][8] = 0;
   coef[6][9] = -1;
   coef[6][10] = 0;
   coef[6][11] = 0;
   coef[6][12] = 0;
   coef[6][13] = 0;
   coef[7][0] = 0;
   coef[7][1] = 0;
   coef[7][2] = 0;
   coef[7][3] = 0;
   coef[7][4] = 0;
   coef[7][5] = 0;
   coef[7][6] = 0;
   coef[7][7] = -1;
   coef[7][8] = -1;
   coef[7][9] = 0;
   coef[7][10] = 0;
   coef[7][11] = 0;
   coef[7][12] = 0;
   coef[7][13] = 0;
   coef[8][0] = 0;
   coef[8][1] = -mR*WR;
   coef[8][2] = 0;
   coef[8][3] = 0;
   coef[8][4] = 0;
   coef[8][5] = 0;
   coef[8][6] = 0;
   coef[8][7] = 0;
   coef[8][8] = 0;
   coef[8][9] = 2;
   coef[8][10] = 0;
   coef[8][11] = 0;
   coef[8][12] = 0;
   coef[8][13] = 0;
   coef[9][0] = mR;
   coef[9][1] = 0;
   coef[9][2] = 0;
   coef[9][3] = 0;
   coef[9][4] = 0;
   coef[9][5] = 0;
   coef[9][6] = 0;
   coef[9][7] = 0;
   coef[9][8] = 0;
   coef[9][9] = 0;
   coef[9][10] = -1;
   coef[9][11] = 0;
   coef[9][12] = 0;
   coef[9][13] = -1;
   coef[10][0] = 0;
   coef[10][1] = 0;
   coef[10][2] = 0;
   coef[10][3] = 0;
   coef[10][4] = 0;
   coef[10][5] = 0;
   coef[10][6] = 0;
   coef[10][7] = 0;
   coef[10][8] = 0;
   coef[10][9] = 0;
   coef[10][10] = 0;
   coef[10][11] = -1;
   coef[10][12] = -1;
   coef[10][13] = 0;
   coef[11][0] = 0;
   coef[11][1] = 0;
   coef[11][2] = 0.5*mR*pow(WR,2);
   coef[11][3] = 1;
   coef[11][4] = 0;
   coef[11][5] = 0;
   coef[11][6] = 0;
   coef[11][7] = 0;
   coef[11][8] = 0;
   coef[11][9] = 0;
   coef[11][10] = 0;
   coef[11][11] = 0;
   coef[11][12] = 0;
   coef[11][13] = -WR;
   coef[12][0] = 1;
   coef[12][1] = 0;
   coef[12][2] = WR;
   coef[12][3] = 0;
   coef[12][4] = 0;
   coef[12][5] = 0;
   coef[12][6] = 0;
   coef[12][7] = 0;
   coef[12][8] = 0;
   coef[12][9] = 0;
   coef[12][10] = 0;
   coef[12][11] = 0;
   coef[12][12] = 0;
   coef[12][13] = 0;
   coef[13][0] = 1;
   coef[13][1] = WR;
   coef[13][2] = 0;
   coef[13][3] = 0;
   coef[13][4] = 0;
   coef[13][5] = 0;
   coef[13][6] = 0;
   coef[13][7] = 0;
   coef[13][8] = 0;
   coef[13][9] = 0;
   coef[13][10] = 0;
   coef[13][11] = 0;
   coef[13][12] = 0;
   coef[13][13] = 0;
}


/* ------------------------------------------------------------------------ */
   void  Simulation::CalculateRhsMatrix( double rhs[], double t, char isIntegratorBoundary )
{
   rhs[0] = 0;
   rhs[1] = -g*mA;
   rhs[2] = cv*qbDt;
   rhs[3] = 0.5*Lb*(mB+2*mQ)*(sin(qb)*pow(qbDt,2)-cos(qb)*qbDDt);
   rhs[4] = -g*(mB+mQ) - 0.5*Lb*(mB+2*mQ)*cos(qb)*pow(qbDt,2) - 0.5*Lb*(mB+2*mQ)*sin(qb)*qbDDt;
   rhs[5] = 0.25*mB*pow(Lb,2)*(mB+4*mQ)*qbDDt/(mB+mQ) - 0.5*g*Lb*(2*mQ*(1-mQ/(mB+mQ))+mB*(1-(mB+3*mQ)/(mB+mQ)))*sin(qb)
      - cv*qbDt - 0.08333333333333333*mB*(pow(Wb,2)+4*pow(Lb,2))*qbDDt - mQ*pow(Lb,2)*(1-mQ/(mB+mQ))*qbDDt;
   rhs[6] = 0;
   rhs[7] = -g*mR;
   rhs[8] = 0;
   rhs[9] = 0;
   rhs[10] = -g*mR;
   rhs[11] = 0;
   rhs[12] = 0;
   rhs[13] = 0;
}


/* ------------------------------------------------------------------------ */
   void  Simulation::UpdateQuantitiesSolvedViaCoupledAlgebraicEquations( double SolutionToAxEqualsB[], double Torque[])
{
   xDDt = SolutionToAxEqualsB[0];
   qcDDt = SolutionToAxEqualsB[1];
   qeDDt = SolutionToAxEqualsB[2];
   Tw = SolutionToAxEqualsB[3];
   FrxAB = SolutionToAxEqualsB[4];
   FryAB = SolutionToAxEqualsB[5];
   FrxAC = SolutionToAxEqualsB[6];
   FryAC = SolutionToAxEqualsB[7];
   FryCF = SolutionToAxEqualsB[8];
   FfxCF = SolutionToAxEqualsB[9];
   FrxAE = SolutionToAxEqualsB[10];
   FryAE = SolutionToAxEqualsB[11];
   FryEF = SolutionToAxEqualsB[12];
   FfxEF = SolutionToAxEqualsB[13];

   Torque[nbLoops] = Tw;
   nbLoops++;
}


/* ------------------------------------------------------------------------ */
   void  Simulation::SetArrayFromVariables( double VAR[] )
{
   VAR[0] = qc;
   VAR[1] = qe;
   VAR[2] = x;
   VAR[3] = qcDt;
   VAR[4] = qeDt;
   VAR[5] = xDt;
}


/* ------------------------------------------------------------------------ */
   void  Simulation::SetDerivativeArray( double VARp[] )
{
   VARp[0] = qcDt;
   VARp[1] = qeDt;
   VARp[2] = xDt;
   VARp[3] = qcDDt;
   VARp[4] = qeDDt;
   VARp[5] = xDDt;
}


/* ------------------------------------------------------------------------ */
   void  Simulation::SetVariablesFromArray( double VAR[] )
{
   qc = VAR[0];
   qe = VAR[1];
   x = VAR[2];
   qcDt = VAR[3];
   qeDt = VAR[4];
   xDt = VAR[5];
}


/* ------------------------------------------------------------------------ */
   std::string  Simulation::MGeqns( double t, double VAR[], double VARp[], char isIntegratorBoundary, double Torque[])
{
   std::string errorMessage = "";
   double SolutionToAxEqualsB[14];

   /* Keep track of number of calls to this function */
   myNumberOfCallsToMGeqns++;

   /* Update named variables from variables passed to function */
   SetVariablesFromArray( VAR );

   /* Calculate specified assigned quantities. */
   CalculateSpecifiedAssignedQuantities( t, isIntegratorBoundary );

   /* Calculate the coefficient and rhs matrices and solve linear algebraic equations. */
   CalculateCoefficientMatrix( COEF, t, isIntegratorBoundary );
   CalculateRhsMatrix( RHS, t, isIntegratorBoundary );
   if( (errorMessage = MGSolveLinearEquation( 14, COEF, RHS, SolutionToAxEqualsB )) != "" ) return errorMessage;
   UpdateQuantitiesSolvedViaCoupledAlgebraicEquations( SolutionToAxEqualsB, Torque);

   /* Update derivative array prior to integration step */
   SetDerivativeArray( VARp );
   return errorMessage;
}

   std::string  Simulation::MGIntegrator( std::function<std::string(double, double*, double*, char, double[])> eqns,
                           int numY, double y[], double tStart, double* hEntry, double* hNext,
                           double smallestAllowableStepsize, double Torque[])
{
   double f0[myNumberOfODES], f1[myNumberOfODES], f2[myNumberOfODES];
   double y1[myNumberOfODES], y2[myNumberOfODES];
   double h = hEntry ? *hEntry : 0;

   /* Always calculate derivatives at tStart (integration boundary here).   */
   /* If h == 0, just call eqns at tStart and return.                       */
   std::string errorMessage = eqns( tStart, y, f0, 1 , Torque);
   if( errorMessage.empty()  &&  h == 0 )  return "";

   /* Avoid endless loop due to adding tiny h to tStart (precision problem) */
   while( errorMessage.empty()  &&  tStart + 0.1*h != tStart )
   {
      int    i, indexFailedVariable = -1;        /* Variable that failed.   */
      double errorRatioMax = 0;                  /* Testing error criterion */
      double h2 = h * 0.5;                                 /* Half    of h  */
      double h3 = h / 3.0;                                 /* Third   of h  */
      double h6 = h / 6.0;                                 /* Sixth   of h  */
      double h8 = h * 0.125;                               /* Eighth  of h  */
      for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h3*f0[i];
      if( (errorMessage = eqns( tStart+h3, y1, f1, 0 , Torque)) != "" ) break;
      for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h6*(f0[i] + f1[i]);
      if( (errorMessage = eqns( tStart+h3, y1, f1, 0, Torque )) != "" ) break;
      for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h8*(f0[i] + 3*f1[i]);
      if( (errorMessage = eqns( tStart+h2, y1, f2, 0, Torque )) != "" ) break;
      for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h2*(f0[i] - 3*f1[i] + 4*f2[i]);
      if( (errorMessage = eqns( tStart+h,  y1, f1, 0, Torque )) != "")  break;
      for( i=0;  i<numY;  i++ )  y2[i] = y[i] + h6*(f0[i] + 4*f2[i] + f1[i]);

      /* Both y1[i] and y2[i] provide estimates to the new value of y.      */
      /* Decide if the relative and absolute error tolerances are met.      */
      /* If they are not, reduce the stepsize and restart the integration.  */
      for( i = 0;  i < numY;  i++ )              /* Check all variables     */
      {
         double errorInEstimate = 0.2 * fabs( y2[i] - y1[i] );
         double relTest  = fabs( y2[i] ) * relError;
         double largerOfAbsErrOrRelTest = absError > relTest ? absError : relTest;
         double errorRatio = errorInEstimate / largerOfAbsErrOrRelTest;
         if( errorRatio > errorRatioMax ) { errorRatioMax = errorRatio;  indexFailedVariable = i; }
      }

      /* If integration succeeded, update values of y and t before return.  */
      /* Return actual stepsize and estimate for next integration stepsize. */
      if( errorRatioMax < 1 )
      {
         for( i = 0;  i < numY;  i++ )  y[i] = y2[i];
         *hEntry = h;   *hNext = (errorRatioMax < 1.0/64.0) ? 2*h : h;
         return "";
      }

      /* Otherwise, errorRatioMax >= 1, so absError or relTest failed.      */
      /* In other words, errorInEstimate >= largerOfAbsErrOrRelTest.        */
      /* Try to halve stepsize and restart integration.                     */
      if( fabs(h=h2) <= fabs(smallestAllowableStepsize) )
      {
            char stepsizeCutMessage[128];
         /*sprintf(errorMessage = "",
               "Error: Numerical integration stepsize cut too many times at t = %17.9E (variable %d).",
               tStart, indexFailedVariable);*/
      }
   }

   /* Check if loop terminated due to numerical round-off.                  */
   if( errorMessage.empty() )
      errorMessage = "Error: Numerical round off makes stepsize h too small relative to tStart, so tStart+h = tStart."
                  "\nIntegration stepsize may have been cut too many times.";

   /* Print error message that numerical integrator failed.    */
   /* If h != 0, call eqns to fill for error display.          */
   printf( "\n Error: Numerical integration failed at t = %17.9E.\n\n", tStart );
   if( h != 0 ) eqns( tStart, y, f0, 1, Torque );
   return errorMessage;
}


/* -------------------------------------------------------------------------- **
** PURPOSE:  Controls the independent variable, e.g., t while solving a set   **
**           of first order ordinary differential equations of the form       **
**           dy(i)/dt = f( t, y(1), ..., y(n) )  (i = 1, ..., n).             **
**           Discontinuities in functions and t should be handled here.       **
**                                                                            **
** INPUT:    eqns, numY, y, absError, relError are described above.           **
**           If numY = 0, the  value of myPreviousStepsize is reset to  **
**           tStep.  This is useful when integrating several sets of ODEs.    **
**                                                                            **
**        t  The current value of the independent variable.                   **
** tStepMax  Maximum integration stepsize for this integrator step.  On entry **
**           if tStepMax = 0, eqns is called and dy(i)/dt (i=1 ... n) are     **
**           evaluated, but no integration performed.                         **
**                                                                            **
** OUTPUT:   Returns an error message if integration failed, otherwise NULL.  **
**        t  The value of  t + tStep  is returned in t.                       **
**        y  The values of y(i) (i=1 ... n) at t + tStep are returned in y.   **
** -------------------------------------------------------------------------- */
   std::string  Simulation::MGIntegrateOneStep( std::function<std::string(double, double*, double*, char, double[])> eqns,
                                 int numY, double y[], double* t, double tStepMax,
                                 double* stepsizeSuggested, double smallestAllowableStepsize, double Torque[])
{
   double hAccumulated = 0;                      /* How far to tStepMax.    */
   double h = *stepsizeSuggested;                /* Current stepsize.       */
   int    isStepFinished = (tStepMax == 0);

   /* Make as many little integration steps as necessary to get to end.     */
   /* Each integration step starts at *t and ends at *t + h.                */
   while( !isStepFinished )
   {
      /* Numerically integrate y[i] and maybe get a smaller value of h.     */
      /* Set hNext to integrator's estimate of next integration step-size.  */
      double hBeforeCall = h, hNext;             /* Suggested stepsize.     */
      std::string errorMessage = MGIntegrator( eqns, numY, y, *t, &h, &hNext, smallestAllowableStepsize, Torque);
      if( !errorMessage.empty() ) return errorMessage;    /* Integration failed      */

      /* Any time or function discontinuities should be handled here.       */
      /* Increment value of t using stepsize returned by the integrator.    */
      *t += h;                                   /* Prepare for next step.  */

      /* Change the stepsize if not taking the last step.                   */
      /* Reduce the stepsize if the integrator reduced the stepsize.        */
      /* Increase the stepsize if the integrator suggests a larger one.     */
      /* Update stepsizeSuggested for next call to integrator.              */
      /* Ensure stepsizeSuggested is not larger than tStepMax.              */
      isStepFinished = fabs( (hAccumulated+=h) + smallestAllowableStepsize) > fabs(tStepMax);
      if( !isStepFinished  ||  fabs(h) < fabs(hBeforeCall)  ||  fabs(hNext) > fabs(*stepsizeSuggested) )
      {
         if( fabs(hNext) > fabs(tStepMax) ) hNext = tStepMax;
         *stepsizeSuggested = h = hNext;
      }

      /* Integrator may suggest a stepsize that is bigger than allowed.     */
      /* If next jump is past or very close to end of tStepMax, adjust h.   */
      if( fabs(hAccumulated + 1.1*h) > fabs(tStepMax) )  h = tStepMax - hAccumulated;
   }

   /* Integration finished.  Calculate derivatives of y at final value of t */
   return MGIntegrator( eqns, numY, y, *t, NULL, NULL, 0, Torque);
}


/* ------------------------------------------------------------------------ */
   std::string  Simulation::MGIntegrateForwardOrBackward( int numVariables, double varArrayToIntegrate[], double outputToFill[], double tInitial, double tFinal, double tStepMax, int printIntScreen, int printIntFile, double Torque[])
{
   double stepsizeSuggested = tStepMax,   smallestAllowableStepsize = 1.0E-7 * tStepMax;
   double t = tInitial,   tFinalMinusEpsilon = tFinal - smallestAllowableStepsize;
   int    isFirstCall = 1,  isIntegrateForward = (tFinal > tInitial),  integrateDirection = isIntegrateForward ? 1 : -1;
   int    isPrintToScreen,  isPrintToFile,  printCounterScreen = 0,   printCounterFile = 0;

   /* Ensure valid parameters and sign(tStepMax) moves integration in proper direction. */
   std::string errorMessage = (numVariables <= 0  ||  !varArrayToIntegrate  ||  absError <= 0  || relError < 0  ||  integrateDirection * tStepMax < 0) ?
                        "Error: Invalid argument to MGIntegrateForwardOrBackward" : "";

   /* Initialize integrator with call at t = tInitial, thereafter integrate */
   int isIntegrationFinished = 0;
   while( !isIntegrationFinished  &&  errorMessage.empty() )
   {
      /* Near the end of numerical integration, perhaps take a partial step (decrease tStepMax). */
      if( (isIntegrateForward && t+tStepMax > tFinal)  ||  (!isIntegrateForward && t+tStepMax < tFinal) )
      {
         tStepMax = tFinal - t;
         if( integrateDirection * stepsizeSuggested > fabs(tStepMax) )  stepsizeSuggested = tStepMax;
      }
      /* Due to round-off or other, maybe set stepsizeSuggested = tStepMax. */
      else if( fabs(stepsizeSuggested) >= 0.99 * fabs(tStepMax) ) stepsizeSuggested = tStepMax;

      errorMessage = MGIntegrateOneStep(std::bind(Simulation::MGeqns, *this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5), 
                                       numVariables, varArrayToIntegrate, &t, (isFirstCall ? 0 : tStepMax), &stepsizeSuggested, smallestAllowableStepsize, Torque);
      isIntegrationFinished = !errorMessage.empty()  ||  (isIntegrateForward && t > tFinalMinusEpsilon)  ||  (!isIntegrateForward && t < tFinalMinusEpsilon);
      isPrintToScreen = printIntScreen  &&  (isIntegrationFinished || --printCounterScreen <= 0);
      isPrintToFile   = printIntFile    &&  (isIntegrationFinished || --printCounterFile   <= 0);
      isFirstCall = 0;
   }
   return errorMessage;
}


/* -------------------------------------------------------------------------- **
** PURPOSE:  The matrix equation A*X = B is solved for X, where A is an       **
**           n by n matrix, and X and B are n by 1 matrices.                  **
**                                                                            **
** INPUT:    numberOfEqns: The integer n (number of equations).               **
**           A: An array of n pointers to rows of doubles.                    **
**              In effect, A is an n by n array of double precision numbers.  **
**           B: An n by 1 array of double precision numbers.                  **
**              The elements of B are unchanged on return.                    **
**                                                                            **
** OUTPUT:   On return, X contains the solution, an n by 1 array of doubles.  **
**           Returns an error message if no solution could be found.          **
**                                                                            **
** Copyright (c) 2009-2019 by Paul Mitiguy.  Use permitted under the 3-clause **
**      BSD license: https://opensource.org/licenses/BSD-3-Clause.            **
**      This copyright notice must appear in all copies & distributions.      **
**      This copyright notice applies to the function MGSolveLinearEquation.  **
** -------------------------------------------------------------------------- */
   std::string  Simulation::MGSolveLinearEquation( int numberOfEqns, double* A[], double B[], double X[] )
{
      char errorMessage[96];
   double      rowScaleFactor[ myNumberOfCoupledLinearEqns ];
   int         i, j, k;

   /* Ensure there is sufficient space for row-scaling factors */
   if( numberOfEqns > myNumberOfCoupledLinearEqns ) return "Error in MGSolveLinearEquation: numberOfEqns > myNumberOfCoupledLinearEqnsIncrease.";

   /* Begin decomposition */
   for( i = 0;  i < numberOfEqns;  i++ )
   {
      /* Find the element in each row with the maximum absolute value */
      double *Ai = A[i];                         /* Short-cut to this row   */
      double rowMax = 0.0;                       /* Max absolute column     */
      for( j = 0;  j < numberOfEqns;  j++ )      /* Check for zero row      */
      if( rowMax < fabs(Ai[j]) )  rowMax = fabs(Ai[j]);

      /* Issue error if row of zeros are found */
      if( rowMax == 0.0 ) { sprintf( errorMessage, "Error in MGSolveLinearEquation: All elements in row %d of coefficient matrix are zero.", i+1 );  return errorMessage; }

      /* Keep track of row scaling factor */
      rowScaleFactor[i] = 1.0 / rowMax;

      /* Keep the B matrix unchanged by copying its elements into x */
      X[i] = B[i];
   }

   /* Change ordering of rows */
   for( k = 0;  k < numberOfEqns;  k++ )
   {
      double largestPivot = 0.0;                 /* Largest relative pivot  */
      int    swapi = k;
      for( i = k;  i < numberOfEqns;  i++ )      /* Check remaining rows    */
      {
         double relSizeThisColumn = fabs(A[i][k]) * rowScaleFactor[i];
         if( relSizeThisColumn > largestPivot ) { swapi = i;  largestPivot = relSizeThisColumn; }
      }

      /* Issue warning if zero pivot is encountered */
      if( largestPivot == 0.0 )  { sprintf( errorMessage, "Error in MGSolveLinearEquation: Zero pivot in column %d during LU-decomposition of COEF matrix.", k+1 );  return errorMessage; }

      /* Maybe switch rows of A and x by changing row pointers and x values */
      if( swapi != k )
      {
         double *swapa = A[k],  swapx = X[k],  swapScale = rowScaleFactor[k];
         A[k] = A[swapi];  A[swapi] = swapa;     /* Change row pointers of A*/
         X[k] = X[swapi];  X[swapi] = swapx;     /* Change row values of X  */
         rowScaleFactor[k] = rowScaleFactor[swapi];  rowScaleFactor[swapi] = swapScale;
      }

      /* Calculate value of pivot and change lower rows */
      if( k < numberOfEqns - 1 )
      {
         double pivotReciprocal = 1.0 / A[k][k];
         for( i = k+1;  i < numberOfEqns;  i++ )
         {
            double *Ak = A[k],  *Ai = A[i];         /* Shortcuts to rows       */
            double ratio = Ai[k] * pivotReciprocal; /* Multiplicative factor   */
            X[i] -= ratio * X[k];                   /* Modify elements of X    */
            for( j = k+1;  j < numberOfEqns;  j++ )  Ai[j] -= ratio * Ak[j];
         }
      }
   }

   /* Solve upperTriangularMatrix * X = Z  */
   for( i = numberOfEqns-1;  i >= 0;  i-- )
   {
      double sum = 0.0, *Ai = A[i];
      for( j = i+1;  j < numberOfEqns;  j++ )  sum += Ai[j] * X[j];
      X[i] = (X[i] - sum) / Ai[i];
   }
   return "";
}


