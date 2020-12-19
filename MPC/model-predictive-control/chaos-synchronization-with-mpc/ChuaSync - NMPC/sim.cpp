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
 
    TIME autotime;
    DifferentialState x1;
    DifferentialState y1;
    DifferentialState z1;
    DifferentialState x2;
    DifferentialState y2;
    DifferentialState z2;
    Control ux;
    Control uy;
    Control uz;
    SIMexport ExportModule1( 1, 0.02 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    ExportModule1.set( INTEGRATOR_TYPE, INT_RK4 );
    ExportModule1.set( NUM_INTEGRATOR_STEPS, 4 );
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x1) == (1/7.000000E+00*x1-2.000000E+00*pow(x1,3.000000E+00)+y1)*1.000000E+01;
    acadodata_f1 << dot(y1) == (x1-y1+z1);
    acadodata_f1 << dot(z1) == (-1.428571E+01)*y1;
    acadodata_f1 << dot(x2) == ((1/7.000000E+00*x2-2.000000E+00*pow(x2,3.000000E+00)+y2)*1.000000E+01+ux);
    acadodata_f1 << dot(y2) == (uy+x2-y2+z2);
    acadodata_f1 << dot(z2) == (1.428571E+01*y2+uz);

    ExportModule1.setModel( acadodata_f1 );

    ExportModule1.setTimingSteps( 0 );
    ExportModule1.exportCode( "export_SIM" );


    clearAllStaticCounters( ); 
 
} 

