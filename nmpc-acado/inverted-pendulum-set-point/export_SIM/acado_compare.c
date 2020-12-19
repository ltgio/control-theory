/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


#include <stdio.h>
#include "acado_auxiliary_sim_functions.h"

/* SOME CONVENIENT DEFINTIONS: */
/* --------------------------------------------------------------- */
   #define h           0.05      /* length of one simulation interval   */
   #define RESULTS_NAME	  "results.txt"
   #define REF_NAME  "ref.txt"
/* --------------------------------------------------------------- */


/* GLOBAL VARIABLES FOR THE ACADO REAL-TIME ALGORITHM: */
/* --------------------------------------------------- */
   ACADOworkspace acadoWorkspace;
   ACADOvariables acadoVariables;



/* A TEMPLATE FOR TESTING THE INTEGRATOR: */
/* ---------------------------------------------------- */
int main(){

   /* INTRODUCE AUXILIARY VAIRABLES: */
   /* ------------------------------ */
      FILE *file, *ref;
      int i, j, nil;
      real_t x[ACADO_NX+ACADO_NXA];
      real_t xRef[ACADO_NX+ACADO_NXA];
      real_t maxErr, meanErr, maxErrX, meanErrX, maxErrXA, meanErrXA, temp;
      const ACADOworkspace nullWork2 = {0};
 	  acadoWorkspace = nullWork2;


   /* START EVALUATION RESULTS: */
   /* ---------------------------------------- */
      meanErrX = 0;
      meanErrXA = 0;
      file = fopen(RESULTS_NAME,"r");
      ref = fopen(REF_NAME,"r");
      for( i = 0; i < (ACADO_NX+ACADO_NXA)*(1+ACADO_NX+ACADO_NU)+1; i++ ) {
      		nil = fscanf( file, "%lf", &temp );
      		nil = fscanf( ref, "%lf", &temp );
      }
      printf( " STATES:\n" );

      for( i = 1; i <= ACADO_N; i++ ) {
      		nil = fscanf( file, "%lf", &temp );
      		nil = fscanf( ref, "%lf", &temp );

      		maxErrX = 0;
      		for( j = 0; j < ACADO_NX; j++ ) {
      			nil = fscanf( file, "%lf", &x[j] );
      			nil = fscanf( ref, "%lf", &xRef[j] );
      			temp = fabs(x[j] - xRef[j])/fabs(xRef[j]);
      			if( temp > maxErrX ) maxErrX = temp;
      			if( isnan(x[j]) ) maxErrX = sqrt(-1);
      		}

      		maxErrXA = 0;
      		for( j = 0; j < ACADO_NXA; j++ ) {
      			nil = fscanf( file, "%lf", &x[ACADO_NX+j] );
      			nil = fscanf( ref, "%lf", &xRef[ACADO_NX+j] );
      			temp = fabs(x[ACADO_NX+j] - xRef[ACADO_NX+j])/fabs(xRef[ACADO_NX+j]);
      			if( temp > maxErrXA ) maxErrXA = temp;
      			if( isnan(x[ACADO_NX+j]) ) maxErrXA = sqrt(-1);
      		}

      		printf( "MAX ERROR AT %.3f s:   %.4e \n", i*h, maxErrX );
			meanErrX += maxErrX;
			meanErrXA += maxErrXA;

      		for( j = 0; j < (ACADO_NX+ACADO_NXA)*(ACADO_NX+ACADO_NU); j++ ) {
      			nil = fscanf( file, "%lf", &temp );
      			nil = fscanf( ref, "%lf", &temp );
      		}
      }
	  meanErrX = meanErrX/ACADO_N;
	  meanErrXA = meanErrXA/ACADO_N;
      printf( "\n" );
      printf( "TOTAL MEAN ERROR:   %.4e \n", meanErrX );
      printf( "\n\n" );

      return 0;
}
