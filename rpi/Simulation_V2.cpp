#include "Simulation_V2.hpp"

Simulation_V2::Simulation_V2(double height)
{
    h = height;
    CalculateConstants();
}

std::vector<double> Simulation_V2::RunSimulation(double init, double duration, int nbPoints, double initQwDt, double initXDt)
{
    //std::cout <<"1: " << Tw << "\n";

    //Init var
    index = 0;
    qc = 0;
    qe = 0;
    x = 0;
    qcDt = initQwDt;
    qeDt = initQwDt;
    xDt = initXDt;
    double tInitial{init}, tFinal{init+duration}, tStep{duration/nbPoints}, absError{1.0E-05}, relError{1.0E-08}, printIntScreenDbl{1}, printIntFileDbl{0};
    char* errorMessage;

    /* Initialize COEF pointers to beginning of each row */
    { int iloop;  for( iloop = 0;  iloop < 14;  iloop++ )  COEF[iloop] = &(_COEF[iloop][0]); }

    /* Numerically integrate. */
    SetArrayFromVariables( varArrayToIntegrate );
    errorMessage = MGIntegrateForwardOrBackward( myNumberOfODES, varArrayToIntegrate, Output, tInitial, tFinal, tStep, absError, relError, (int)printIntScreenDbl, (int)printIntFileDbl );
    //std::cout <<"2: " << Tw << "\n";
    return Sequence;
}

char* Simulation_V2::OutputToScreenOrFile(double t, double Output[], int isPrintToScreen, int isPrintToFile)
{
    //std::cout <<"29: " << Tw << "\n";
    char* errorMessage;// = OpenOutputFilesWriteHeaders( isPrintToScreen, isPrintToFile );
    CalculateOutput(t, Output);
    //OutputFormattedNumbers( stdout,    Output, 1, 1, 0 );
    //std::cout <<"30: " << Tw << "\n";
    return errorMessage;
}

void Simulation_V2::OutputFormattedNumbers(FILE *Fptr, double *Output, ...)
{
    //std::cout <<"35: " << Tw << "\n";
    va_list varArgList;   va_start( varArgList, Output );
    { int numBlankSpaces;  while( (numBlankSpaces = va_arg(varArgList, int)) != 0 ) fprintf(Fptr, "%*s%- 14.6E", numBlankSpaces, " ", *Output++); }
    va_end( varArgList );
    fprintf( Fptr, "\n" );
    //std::cout <<"36: " << Tw << "\n";
}

char* Simulation_V2::OpenOutputFilesWriteHeaders(int isPrintToScreen, int isPrintToFile)
{
    //std::cout <<"31: " << Tw << "\n";
    char* errorMessage = NULL;
    static int myHasHeaderInfoBeenWritten = 0;
    if( !myHasHeaderInfoBeenWritten && (isPrintToScreen || isPrintToFile) )
    {
        const char* headerFile[1];
        headerFile[0] = "% FILE: Projet.1\n%\n"
                        "%       t             Tw\n"
                        "%     (sec)          (N*m)\n\n";

        if( isPrintToScreen ) fputs( headerFile[0], stdout );
        myHasHeaderInfoBeenWritten = 1;
    }
    //std::cout <<"32: " << Tw << "\n";
    return errorMessage;
}

void Simulation_V2::CalculateSpecifiedAssignedQuantities(double t, char isIntegratorBoundary)
{
    //std::cout <<"17: " << Tw << "\n";
    qb = Amp*t*sin(7.853981633974483*t/Tt)/Tt;
    qbDt = Amp*(Tt*sin(7.853981633974483*t/Tt)+7.853981633974483*t*cos(7.853981633974483*t/Tt))/pow(Tt,2);
    qbDDt = 15.70796326794897*Amp*(cos(7.853981633974483*t/Tt)-3.926990816987241*t*sin(7.853981633974483*t/Tt)/Tt)/pow(Tt,2);
    //std::cout <<"18: " << Tw << "\n";
}

void Simulation_V2::CalculateCoefficientMatrix(double *coef[], double t, char isIntegratorBoundary)
{
    //std::cout <<"19: " << Tw << "\n";
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
    //std::cout <<"20: " << Tw << "\n";
}

void Simulation_V2::CalculateRhsMatrix(double rhs[], double t, char isIntegratorBoundary)
{
    //std::cout <<"21: " << Tw << "\n";
    rhs[0] = 0;
    rhs[1] = -g*mA;
    rhs[2] = cv*qbDt;
    rhs[3] = 0.5*Lb*(mB+2*mQ)*(sin(qb)*pow(qbDt,2)-cos(qb)*qbDDt);
    rhs[4] = -g*(mB+mQ) - 0.5*Lb*(mB+2*mQ)*cos(qb)*pow(qbDt,2) - 0.5*Lb*(mB+2*mQ)*sin(qb)*qbDDt;
    rhs[5] = 0.25*mB*pow(Lb,2)*(mB+4*mQ)*qbDDt/(mB+mQ) - 0.5*g*Lb*(2*mQ*(1-mQ/(mB+mQ))+mB*(1-(mB+3*mQ)/(mB+mQ)))*sin(qb) - cv*qbDt - 0.08333333333333333*mB*(pow(Wb,2)+4*pow(Lb,2))*qbDDt - mQ*pow(Lb,2)*(1-mQ/(mB+mQ))*qbDDt;
    rhs[6] = 0;
    rhs[7] = -g*mR;
    rhs[8] = 0;
    rhs[9] = 0;
    rhs[10] = -g*mR;
    rhs[11] = 0;
    rhs[12] = 0;
    rhs[13] = 0;
    //std::cout <<"22: " << Tw << "\n";
}

void Simulation_V2::UpdateQuantitiesSolvedViaCoupledAlgebraicEquations(double SolutionToAxEqualsB[])
{  
    //std::cout <<"25: " << Tw << "\n";
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
    //std::cout <<"26: " << Tw << "\n";
}

void  Simulation_V2::SetArrayFromVariables( double VAR[] )
{
    //std::cout <<"5: " << Tw << "\n";
   VAR[0] = qc;
   VAR[1] = qe;
   VAR[2] = x;
   VAR[3] = qcDt;
   VAR[4] = qeDt;
   VAR[5] = xDt;
   //std::cout <<"6: " << Tw << "\n";
}

void  Simulation_V2::SetDerivativeArray( double VARp[] )
{
    //std::cout <<"27: " << Tw << "\n";
   VARp[0] = qcDt;
   VARp[1] = qeDt;
   VARp[2] = xDt;
   VARp[3] = qcDDt;
   VARp[4] = qeDDt;
   VARp[5] = xDDt;
   //std::cout <<"28: " << Tw << "\n";
}

void  Simulation_V2::SetVariablesFromArray( double VAR[] )
{
    //std::cout <<"15: " << Tw << "\n";
   qc = VAR[0];
   qe = VAR[1];
   x = VAR[2];
   qcDt = VAR[3];
   qeDt = VAR[4];
   xDt = VAR[5];
   //std::cout <<"16: " << Tw << "\n";
}

char*  Simulation_V2::MGeqns( double t, double VAR[], double VARp[], char isIntegratorBoundary )
{
    //std::cout <<"13: " << Tw << "\n";
    char* errorMessage = NULL;
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
    if( (errorMessage = MGSolveLinearEquation( 14, COEF, RHS, SolutionToAxEqualsB )) != NULL ) return errorMessage;
    UpdateQuantitiesSolvedViaCoupledAlgebraicEquations( SolutionToAxEqualsB );

    /* Update derivative array prior to integration step */
    SetDerivativeArray( VARp );
    //std::cout <<"14: " << Tw << "\n";
    return errorMessage;
}

void Simulation_V2::CalculateOutput(double t, double Output[])
{
    //std::cout <<"33: " << Tw << "\n";
    Output[0] = t;
    Output[1] = qeDt;
    Sequence.push_back(Output[1]);
}

char* Simulation_V2::MGIntegrator(std::function<char *(double, double *, double *, char)> eqns, int numY, double y[], double tStart, double *hEntry, double *hNext, double smallestAllowableStepsize, double absError, double relError)
{
    //std::cout <<"11: " << Tw << "\n";
    double f0[myNumberOfODES], f1[myNumberOfODES], f2[myNumberOfODES];
    double y1[myNumberOfODES], y2[myNumberOfODES];
    double h = hEntry ? *hEntry : 0;

    /* Always calculate derivatives at tStart (integration boundary here).   */
    /* If h == 0, just call eqns at tStart and return.                       */
    //std::cout << "PREMIER CALL\n";
    char* errorMessage = MGeqns( tStart, y, f0, 1 );

    if( !errorMessage  &&  h == 0 )  return NULL;

    /* Avoid endless loop due to adding tiny h to tStart (precision problem) */
    while( !errorMessage  &&  tStart + 0.1*h != tStart )
    {
        int    i, indexFailedVariable = -1;        /* Variable that failed.   */
        double errorRatioMax = 0;                  /* Testing error criterion */
        double h2 = h * 0.5;                                 /* Half    of h  */
        double h3 = h / 3.0;                                 /* Third   of h  */
        double h6 = h / 6.0;                                 /* Sixth   of h  */
        double h8 = h * 0.125;                               /* Eighth  of h  */
        for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h3*f0[i];
        //std::cout << "DEUXIEME CALL\n";
        if( (errorMessage = MGeqns( tStart+h3, y1, f1, 0 )) != NULL ) break;
        for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h6*(f0[i] + f1[i]);
        //std::cout << "TROISIEME CALL\n";
        if( (errorMessage = MGeqns( tStart+h3, y1, f1, 0 )) != NULL ) break;
        for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h8*(f0[i] + 3*f1[i]);
        //std::cout << "QUATRIEME CALL\n";
        if( (errorMessage = MGeqns( tStart+h2, y1, f2, 0 )) != NULL ) break;
        for( i=0;  i<numY;  i++ )  y1[i] = y[i] + h2*(f0[i] - 3*f1[i] + 4*f2[i]);
        //std::cout << "CINQUIEME CALL\n";
        if( (errorMessage = MGeqns( tStart+h,  y1, f1, 0 )) != NULL)  break;
        for( i=0;  i<numY;  i++ )  y2[i] = y[i] + h6*(f0[i] + 4*f2[i] + f1[i]);
        //std::cout <<"124: " << Tw << "\n";

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
        //std::cout <<"12: " << Tw << "\n";
        if( errorRatioMax < 1 )
        {
            for( i = 0;  i < numY;  i++ )  y[i] = y2[i];
            *hEntry = h;   *hNext = (errorRatioMax < 1.0/64.0) ? 2*h : h;
            return NULL;
        }

        /* Otherwise, errorRatioMax >= 1, so absError or relTest failed.      */
        /* In other words, errorInEstimate >= largerOfAbsErrOrRelTest.        */
        /* Try to halve stepsize and restart integration.                     */
        if( fabs(h=h2) <= fabs(smallestAllowableStepsize) )
        {
            static char stepsizeCutMessage[128];
            sprintf(errorMessage = stepsizeCutMessage,
                    "Error: Numerical integration stepsize cut too many times at t = %17.9E (variable %d).",
                    tStart, indexFailedVariable);
        }
    }

    /* Check if loop terminated due to numerical round-off.                  */
    if( !errorMessage )
        errorMessage = "Error: Numerical round off makes stepsize h too small relative to tStart, so tStart+h = tStart."
                    "\nIntegration stepsize may have been cut too many times.";

    /* Print error message that numerical integrator failed.    */
    /* If h != 0, call eqns to fill for error display.          */
    printf( "\n Error: Numerical integration failed at t = %17.9E.\n\n", tStart );
    //std::cout << "SIXIEME CALL\n";
    if( h != 0 ) MGeqns( tStart, y, f0, 1 );

    return errorMessage;
}

char *Simulation_V2::MGIntegrateOneStep(std::function<char *(double, double *, double *, char)> eqns, int numY, double y[], double *t, double tStepMax, double *stepsizeSuggested, double smallestAllowableStepsize, double absError, double relError)
{
    //std::cout <<"9: " << Tw << "\n";
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
        char* errorMessage = MGIntegrator( eqns, numY, y, *t, &h, &hNext, smallestAllowableStepsize, absError, relError );
        if( errorMessage ) return errorMessage;    /* Integration failed      */

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
    //std::cout <<"10: " << Tw << "\n";
    return MGIntegrator( eqns, numY, y, *t, NULL, NULL, 0, 0, 0 );
}

char* Simulation_V2::MGIntegrateForwardOrBackward(int numVariables, double varArrayToIntegrate[], double OutputToFill[], double tInitial, double tFinal, double tStepMax, double absError, double relError, int printIntScreen, int printIntFile)
{
    //std::cout <<"7: " << Tw << "\n";
    double stepsizeSuggested = tStepMax,   smallestAllowableStepsize = 1.0E-7 * tStepMax;
    double t = tInitial,   tFinalMinusEpsilon = tFinal - smallestAllowableStepsize;
    int    isFirstCall = 1,  isIntegrateForward = (tFinal > tInitial),  integrateDirection = isIntegrateForward ? 1 : -1;
    int    isPrintToScreen,  isPrintToFile,  printCounterScreen = 0,   printCounterFile = 0;

    /* Ensure valid parameters and sign(tStepMax) moves integration in proper direction. */
    char* errorMessage = (numVariables <= 0  ||  !varArrayToIntegrate  ||  absError <= 0  || relError < 0  ||  integrateDirection * tStepMax < 0) ?
                            (char*)"Error: Invalid argument to MGIntegrateForwardOrBackward" : NULL;

    /* Initialize integrator with call at t = tInitial, thereafter integrate */
    int isIntegrationFinished = 0;
    while( !isIntegrationFinished  &&  !errorMessage )
    {
        /* Near the end of numerical integration, perhaps take a partial step (decrease tStepMax). */
        if( (isIntegrateForward && t+tStepMax > tFinal)  ||  (!isIntegrateForward && t+tStepMax < tFinal) )
        {
            tStepMax = tFinal - t;
            if( integrateDirection * stepsizeSuggested > fabs(tStepMax) )  stepsizeSuggested = tStepMax;
        }
        /* Due to round-off or other, maybe set stepsizeSuggested = tStepMax. */
        else if( fabs(stepsizeSuggested) >= 0.99 * fabs(tStepMax) ) stepsizeSuggested = tStepMax;
        errorMessage = MGIntegrateOneStep( std::bind(Simulation_V2::MGeqns, *this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), numVariables, varArrayToIntegrate, &t, (isFirstCall ? 0 : tStepMax), &stepsizeSuggested, smallestAllowableStepsize, absError, relError );
        isIntegrationFinished = errorMessage  ||  (isIntegrateForward && t > tFinalMinusEpsilon)  ||  (!isIntegrateForward && t < tFinalMinusEpsilon);
        isPrintToScreen = printIntScreen  &&  (isIntegrationFinished || --printCounterScreen <= 0);
        isPrintToFile   = printIntFile    &&  (isIntegrationFinished || --printCounterFile   <= 0);
        if( isPrintToScreen || isPrintToFile )
        {
            char* errorMessage2 = OutputToScreenOrFile( t, OutputToFill, isPrintToScreen, isPrintToFile );
            if( errorMessage2 && !errorMessage ) errorMessage = errorMessage2;
            if( isPrintToScreen ) printCounterScreen = printIntScreen;
            if( isPrintToFile   ) printCounterFile   = printIntFile;
        }
        isFirstCall = 0;
    }
    //std::cout <<"8: " << Tw << "\n";
    return errorMessage;
}

void Simulation_V2::CalculateConstants(void)
{
    //std::cout <<"3: " << Tw << "\n";
    Freq = sqrt(g/Lb-pow(cv,2));
    Tt = 7.853981633974483/Freq;
    Amp = acos(1-h/Lb);
    //std::cout <<"4: " << Tw << "\n";
}

char* Simulation_V2::MGSolveLinearEquation( int numberOfEqns, double* A[], double B[], double X[] )
{
    //std::cout <<"23: " << Tw << "\n";
    static char errorMessage[96];
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
    //std::cout <<"24: " << Tw << "\n";
    return NULL;
}
