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
#ifndef SIMULATION_V2_HPP
#define SIMULATION_V2_HPP
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <functional>
#include <vector>
#define  myNumberOfCoupledLinearEqns 14
#define  myNumberOfODES 6


class Simulation_V2
{
    std::vector<double> Sequence;
    int index;
    double CarHeight{0.15}, CarLength{0.2}, cv{0.02}, g {9.81}, h{0.04}, Lb{0.3}, mA{3}, mB{0.2}, mQ{1}, mR{0.5}, Wb{0.03}, WR{0.05};
    double qc{0}, qe{0}, x{0}, qcDt{0}, qeDt{0}, xDt{0};
    double FfxCF, FfxEF, FrxAB, FrxAC, FrxAE, FryAB, FryAC, FryAE, FryCF, FryEF, Tw, qcDDt, qeDDt, xDDt, qb, qbDt, qbDDt, Amp, Freq, Tt;
    double Pi = 3.141592653589793, _COEF[14][14], *COEF[14], RHS[14], varArrayToIntegrate[myNumberOfODES], Output[2];
    double DEGtoRAD = 0.0174532925199432957692369;
    unsigned long myNumberOfCallsToMGeqns = 0ul;

    public:
        Simulation_V2(double height);
         std::vector<double> RunSimulation(double init, double duration, int nbPoints, double initQwDt, double initXDt);
    private:
        char*  OutputToScreenOrFile( double t, double Output[], int isPrintToScreen, int isPrintToFile );
        void   OutputFormattedNumbers( FILE* Fptr, double* Output, ... );
        char*  OpenOutputFilesWriteHeaders( int isPrintToScreen, int isPrintToFile );
        void   CalculateSpecifiedAssignedQuantities( double t, char isIntegratorBoundary );
        void   CalculateCoefficientMatrix( double *coef[], double t, char isIntegratorBoundary );
        void   CalculateRhsMatrix( double rhs[], double t, char isIntegratorBoundary );
        void   UpdateQuantitiesSolvedViaCoupledAlgebraicEquations( double SolutionToAxEqualsB[] );
        void   SetArrayFromVariables( double VAR[] );
        void   SetDerivativeArray( double VARp[] );
        void   SetVariablesFromArray( double VAR[] );
        char*  MGeqns( double t, double VAR[], double VARp[], char isIntegratorBoundary );
        void   CalculateOutput( double t, double Output[] );
        char*  MGIntegrator( std::function<char*(double, double*, double*, char)> eqns,int numY, double y[], double tStart, double* hEntry, double* hNext,double smallestAllowableStepsize, double absError, double relError );
        char*  MGIntegrateOneStep( std::function<char*(double, double*, double*, char)> eqns,int numY, double y[], double* t, double tStepMax,double* stepsizeSuggested, double smallestAllowableStepsize,double absError, double relError );
        char*  MGIntegrateForwardOrBackward( int numVariables, double varArrayToIntegrate[], double OutputToFill[], double tInitial, double tFinal, double tStep, double absError, double relError, int printIntScreen, int printIntFile );
        void   CalculateConstants( void );
        char*  MGSolveLinearEquation( int numberOfEqns, double* A[], double B[], double X[] );
};
#endif