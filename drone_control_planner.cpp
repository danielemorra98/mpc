/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
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
 *    \file   examples/ocp/active_damping.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState x_drone("position",1,3);
	DifferentialState v_drone("velocity",1,3);

	Disturbance f_e("external_force",1,3);

	Control x_desired("position",1,3);
  Control v_desired("velocity",1,3);
  Control a_desired("acceleration",1,3);

	IntermediateState thrust;
	IntermediateState phi;
	IntermediateState theta;

	double M   = 1.0;
	double K  = 1.0;
	double D = 4.0;
	double eps = 1e-5;			// for numeric error
	double x_ref = 1, y_ref = 1;	// for the vertical reference trajectory
	DVector g;							// gravity vector
	g.setAll(0.0); 
	g(3)=9.81;
	DVector x0, v0;							// initial state
	x0.setAll(0.0), v0.setAll(0.0);


	// DEFINE A THRUST AND EULER ANGLES EQUATION (see Ruggiero's paper):
	// -------------------------------
	Expression s = M*(a_desired - D/M*(v_desired-v_drone) - K/M*(x_desired-x_drone) - g);
	thrust = sqrt(pow(s),2);
	phi = asin(-s(2)/thrust);
	theta = atan(s(1)/s(3));
	IntermediateState R_rot(3,3);
	R_rot(1,1) = cos(theta), R_rot(1,2) = sin(phi)*sin(theta), R_rot(1,3) = cos(phi)*sin(theta);
	R_rot(2,1) = 0, R_rot(2,2) = cos(phi), R_rot(2,3) = -sin(phi);
	R_rot(3,1) = -sin(theta), R_rot(3,2) = cos(theta)*sin(phi), R_rot(3,3) = cos(phi)*cos(theta);
	IntermediateState a_drone = a_desired + D/M*(v_desired-v_drone)+K/M*(x_desired-x_drone)+1/M*R_rot*f_e;

	// DEFINE A DIFFERENTIAL EQUATION:
	// -------------------------------
	DifferentialEquation f;

	f << dot(x_drone)  == v_drone;
	f << dot(v_drone) == a_drone;



	// DEFINE LEAST SQUARE FUNCTION:
	// -----------------------------
	double q_dist = 1;
	double q_progression = 1;
	double q_smoothness = 0.1;

	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	const double t_start = 0.0;
	const double t_end   = 20.0;

	OCP ocp( t_start, t_end, 20 );

	ocp.minimizeLagrangeTerm( 0.5*(q_dist*(pow(x_drone(1)-x_ref,2)+pow(x_drone(2)-y_ref,2)) + 
		q_progression*v_drone(3) + q_smoothness*pow(a_drone,2) ));

	ocp.subjectTo( f );

	ocp.subjectTo( AT_START, x_drone  == x0 );
	ocp.subjectTo( AT_START, v_drone  == v0 );

	ocp.subjectTo( f_e(1) == 0 ), ocp.subjectTo( f_e(2) == 0 ), ocp.subjectTo( f_e(3) == 0 );


	// Additionally, flush a plotting object
	GnuplotWindow window1;//( PLOT_AT_EACH_ITERATION );
	window1.addSubplot( x_drone(1), "X Position [m]" );
	window1.addSubplot( x_drone(2),"Y Position [m]" );
	window1.addSubplot( x_drone(3), "Z Velocity [m/s]" );


	// DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
	// ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp);

	algorithm << window1;

 // algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
 //  algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON_WITH_BLOCK_BFGS );

	//algorithm.set( INTEGRATOR_TOLERANCE, 1e-8 );
	algorithm.set( KKT_TOLERANCE, 1e-6 );
	//algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );
	//algorithm.set( MAX_NUM_ITERATIONS, 1 );

	algorithm.solve();


	return 0;
}



