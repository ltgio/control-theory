/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    DifferentialState x1;
    DifferentialState y1;
    DifferentialState z1;
    DifferentialState x2;
    DifferentialState y2;
    DifferentialState z2;
    Control ux;
    Control uy;
    Control uz;
    BMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << x1;
    acadodata_f2 << y1;
    acadodata_f2 << z1;
    acadodata_f2 << x2;
    acadodata_f2 << y2;
    acadodata_f2 << z2;
    acadodata_f2 << ux;
    acadodata_f2 << uy;
    acadodata_f2 << uz;
    Function acadodata_f3;
    acadodata_f3 << x1;
    acadodata_f3 << y1;
    acadodata_f3 << z1;
    acadodata_f3 << x2;
    acadodata_f3 << y2;
    acadodata_f3 << z2;
    OCP ocp1(0, 0.8, 40);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo((-1.500000E+00) <= ux <= 1.500000E+00);
    ocp1.subjectTo((-1.500000E+00) <= uy <= 1.500000E+00);
    ocp1.subjectTo((-1.500000E+00) <= uz <= 1.500000E+00);
    DifferentialEquation acadodata_f4;
    acadodata_f4 << dot(x1) == (1/7.000000E+00*x1-2.000000E+00*pow(x1,3.000000E+00)+y1)*1.000000E+01;
    acadodata_f4 << dot(y1) == (x1-y1+z1);
    acadodata_f4 << dot(z1) == (-1.428571E+01)*y1;
    acadodata_f4 << dot(x2) == ((1/7.000000E+00*x2-2.000000E+00*pow(x2,3.000000E+00)+y2)*1.000000E+01+ux);
    acadodata_f4 << dot(y2) == (uy+x2-y2+z2);
    acadodata_f4 << dot(z2) == (1.428571E+01*y2+uz);

    ocp1.setModel( acadodata_f4 );


    OCPexport ExportModule2( ocp1 );
    ExportModule2.set( GENERATE_MATLAB_INTERFACE, 1 );
    ExportModule2.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    ExportModule2.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    ExportModule2.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    ExportModule2.set( INTEGRATOR_TYPE, INT_RK4 );
    ExportModule2.set( NUM_INTEGRATOR_STEPS, 80 );
    ExportModule2.set( QP_SOLVER, QP_QPOASES );
    ExportModule2.set( GENERATE_SIMULINK_INTERFACE, YES );
    ExportModule2.exportCode( "export_MPC" );


    clearAllStaticCounters( ); 
 
} 

