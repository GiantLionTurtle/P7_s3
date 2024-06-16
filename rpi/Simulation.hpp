#ifndef P7_SIMULATION_HPP_
#define P7_SIMULATION_HPP_

#include <functional>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

#define myNumberOfCoupledLinearEqns 14
#define myNumberOfODES 6
#define Pi std::numbers::pi
#define DegtoRAD 0.0174532925199432957692369
#define relError 1e-008
#define absError 1e-008


class Simulation 
{
    int nbLoops {0};

    double CarHeight {0.15}, CarLength {0.2}, cv {0.02}, g {9.81}, h, Lb {0.3}, mA {3}, mB {0.2}, mQ {1}, mR {0.5}, Wb {0.03}, WR {0.05};
    double qc {0}, qe {0}, x {0}, qcDt {0}, qeDt {0}, xDt {0};
    double FfxCF, FfxEF, FrxAB, FrxAC, FrxAE, FryAB, FryAC, FryAE, FryCF, FryEF, Tw, qcDDt, qeDDt, xDDt, qb, qbDt, qbDDt, Amp, Freq, Tt;
    double _COEF[14][14], *COEF[14], RHS[14], varArrayToIntegrate[myNumberOfODES], Output[2];
    double DEGtoRAD = 0.0174532925199432957692369;
    unsigned long myNumberOfCallsToMGeqns = 0ul;


    /* ------------------------------------------------------------------------ */
    public:
    void Simulate(double tInitial, float height, double Torque[100]);

    /* ------------------------------------------------------------------------ */
     void  CalculateConstants( void );


    /* ------------------------------------------------------------------------ */
     void  CalculateSpecifiedAssignedQuantities( double t, char isIntegratorBoundary );


    /* ------------------------------------------------------------------------ */
     void  CalculateCoefficientMatrix( double *coef[], double t, char isIntegratorBoundary );


    /* ------------------------------------------------------------------------ */
     void  CalculateRhsMatrix( double rhs[], double t, char isIntegratorBoundary );


    /* ------------------------------------------------------------------------ */
     void  UpdateQuantitiesSolvedViaCoupledAlgebraicEquations( double SolutionToAxEqualsB[],double Torque[] );


    /* ------------------------------------------------------------------------ */
     void  SetArrayFromVariables( double VAR[] );


    /* ------------------------------------------------------------------------ */
     void  SetDerivativeArray( double VARp[] );


    /* ------------------------------------------------------------------------ */
     void  SetVariablesFromArray( double VAR[] );


    /* ------------------------------------------------------------------------ */
     std::string  MGeqns( double t, double VAR[], double VARp[], char isIntegratorBoundary, double Torque[] );

    
    /* -------------------------------------------------------------------------- */
     std::string  MGIntegrator(std::function<std::string(double, double*, double*, char, double[])> eqns,
                                int numY, double y[], double tStart, double* hEntry, double* hNext,
                                double smallestAllowableStepsize, double Torque[]);


    /* -------------------------------------------------------------------------- */
     std::string  MGIntegrateOneStep( std::function<std::string(double, double*, double*, char, double[])> eqns,
                                    int numY, double y[], double* t, double tStepMax,
                                    double* stepsizeSuggested, double smallestAllowableStepsize, double Torque[]);


    /* ------------------------------------------------------------------------ */
     std::string  MGIntegrateForwardOrBackward( int numVariables, double varArrayToIntegrate[], double outputToFill[], double tInitial, double tFinal, double tStepMax, int printIntScreen, int printIntFile, double Torque[] );


    /* -------------------------------------------------------------------------- */
     std::string  MGSolveLinearEquation( int numberOfEqns, double* A[], double B[], double X[] );
};


#endif