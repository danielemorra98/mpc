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
#include <nlohmann/json.hpp>



int main( ){

	USING_NAMESPACE_ACADO


	// INTRODUCE THE VARIABLES:
	// -------------------------
	DifferentialState pos_x;
	DifferentialState pos_y;
	DifferentialState pos_z;
	DifferentialState v_x;
	DifferentialState v_y;
	DifferentialState v_z;

	Disturbance f_e_x, f_e_y, f_e_z;

	Control pos_desired_x, pos_desired_y, pos_desired_z;
  Control v_desired_x, v_desired_y, v_desired_z;
  Control a_desired_x, a_desired_y, a_desired_z;

	IntermediateState thrust;
	IntermediateState phi;
	IntermediateState theta;

	double M   = 1.0;
	double K  = 1.0;
	double D = 4.0;
	double eps = 1e-5;			// for numeric error
	double x_ref = 1, y_ref = 1;	// for the vertical reference trajectory
	DVector g(3);							// gravity vector
	g.setAll(0.0); 
	g(2)=9.81;
	double pos_x0 = 0;
	double pos_y0 = 0; 
	double pos_z0 = 0;
	double v_x0 = 0; 
	double v_y0 = 0; 
	double v_z0 = 0;							// initial state


	// DEFINE A THRUST AND EULER ANGLES EQUATION (see Ruggiero's paper):
	// -------------------------------
	Expression s_x = M*(a_desired_x - D/M*(v_desired_x-v_x) - K/M*(pos_desired_x-pos_x) - g(0));
	Expression s_y = M*(a_desired_y - D/M*(v_desired_y-v_y) - K/M*(pos_desired_y-pos_y) - g(1));
	Expression s_z = M*(a_desired_z - D/M*(v_desired_z-v_z) - K/M*(pos_desired_z-pos_z) - g(2));

	thrust = (s_x.getPow(2)+s_y.getPow(2)+s_z.getPow(2)).getSqrt();
	phi = asin(-s_y/thrust);
	theta = atan(s_x/s_z);
	IntermediateState R_rot_x(3), R_rot_y(3), R_rot_z(3);
	R_rot_x(0) = cos(theta), R_rot_x(1) = sin(phi)*sin(theta), R_rot_x(2) = cos(phi)*sin(theta);
	R_rot_y(0) = 0, R_rot_y(1) = cos(phi), R_rot_y(2) = -sin(phi);
	R_rot_z(0) = -sin(theta), R_rot_z(1) = cos(theta)*sin(phi), R_rot_z(2) = cos(phi)*cos(theta);
	IntermediateState a_drone_x = a_desired_x + D/M*(v_desired_x-v_x)+K/M*(pos_desired_x-pos_x)+1/M*(R_rot_x(0)*f_e_x+R_rot_x(1)*f_e_y+R_rot_x(2)*f_e_z);
	IntermediateState a_drone_y = a_desired_y + D/M*(v_desired_y-v_y)+K/M*(pos_desired_y-pos_y)+1/M*(R_rot_y(0)*f_e_x+R_rot_y(1)*f_e_y+R_rot_y(2)*f_e_z);
	IntermediateState a_drone_z = a_desired_z + D/M*(v_desired_z-v_z)+K/M*(pos_desired_z-pos_z)+1/M*(R_rot_z(0)*f_e_x+R_rot_z(1)*f_e_y+R_rot_z(2)*f_e_z);

	// DEFINE A DIFFERENTIAL EQUATION:
	// -------------------------------
	DifferentialEquation f;

	f << dot(pos_x)  == v_x;
	f << dot(pos_y)  == v_y;
	f << dot(pos_z)  == v_z;
	f << dot(v_x)  == a_desired_x + D/M*(v_desired_x-v_x)+K/M*(pos_desired_x-pos_x)+1/M*(R_rot_x(0)*f_e_x+R_rot_x(1)*f_e_y+R_rot_x(2)*f_e_z);
	f << dot(v_y)  == a_desired_y + D/M*(v_desired_y-v_y)+K/M*(pos_desired_y-pos_y)+1/M*(R_rot_y(0)*f_e_x+R_rot_y(1)*f_e_y+R_rot_y(2)*f_e_z);
	f << dot(v_z)  == a_desired_z + D/M*(v_desired_z-v_z)+K/M*(pos_desired_z-pos_z)+1/M*(R_rot_z(0)*f_e_x+R_rot_z(1)*f_e_y+R_rot_z(2)*f_e_z);


	// DEFINE WEIGHT OF OBJECTIVE FUNCTION:
	// -----------------------------
	std::ifstream strWeight("weights.json");
	nlohmann::json jsonWeights;
	strWeight >> jsonWeights;
	double q_dist = jsonWeights["q_dist"];
	double q_progression = jsonWeights["q_progression"];
	double q_smoothness = jsonWeights["q_smoothness"];

	// DEFINE FINAL CONDITION:
	// -----------------------------

	Function hN;

	hN << pos_x
		 << pos_y
		 << pos_z;

	double pos_x_final = x_ref;
	double pos_y_final = y_ref;
	double pos_z_final = 2;

	DVector rN(hN.getDim());
	rN(0) = pos_x_final;
	rN(1) = pos_y_final;
	rN(2) = pos_z_final;

	DMatrix QN(hN.getDim(),hN.getDim());
	double q_final_condition = jsonWeights["q_final_condition"];
	QN.setIdentity();
	QN = q_final_condition*QN;


	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	const double t_start = 0.0;
	const double t_end   = 20.0;

	OCP ocp( t_start, t_end, 20 );

	ocp.minimizeLagrangeTerm( 0.5*(q_dist*((pos_x-x_ref).getPow(2)+(pos_y-y_ref).getPow(2)) - 
		q_progression*v_z + q_smoothness*(pow(a_drone_x,2)+pow(a_drone_y,2)+pow(a_drone_z,2) )));

	ocp.minimizeLSQEndTerm(QN, hN, rN);

	ocp.subjectTo( f );

	ocp.subjectTo( AT_START, pos_x  == pos_x0 );
	ocp.subjectTo( AT_START, pos_y  == pos_y0 );
	ocp.subjectTo( AT_START, pos_z  == pos_z0 );
	ocp.subjectTo( AT_START, v_x  == v_x0 );
	ocp.subjectTo( AT_START, v_y  == v_y0 );
	ocp.subjectTo( AT_START, v_z  == v_z0 );

	double thrust_max = 20;
	ocp.subjectTo( thrust <= thrust_max);
	ocp.subjectTo( pos_z >= 0);


	ocp.subjectTo( f_e_x == 0 ), ocp.subjectTo( f_e_y == 0 ), ocp.subjectTo( f_e_z == 0 );


	// Additionally, flush a plotting object
	GnuplotWindow window1;//( PLOT_AT_EACH_ITERATION );
	window1.addSubplot( pos_x, "X Position [m]" );
	window1.addSubplot( pos_y,"Y Position [m]" );
	window1.addSubplot( pos_z, "Z Position [m]" );
	window1.addSubplot( v_x, "X Velocity [m/s]" );
	window1.addSubplot( v_y,"Y Velocity [m/s]" );
	window1.addSubplot( v_z, "Z Velocity [m/s]" );
	window1.addSubplot( thrust, "Thrust [m/s^2]" );
	window1.addSubplot( phi*180/M_PI,"Phi angle [degree]" );
	window1.addSubplot( theta*180/M_PI, "Theta angle [degree]" );



	// DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
	// ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp);

	algorithm << window1;

 	// algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
 	// algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	// algorithm.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON_WITH_BLOCK_BFGS );

	//algorithm.set( INTEGRATOR_TOLERANCE, 1e-8 );
	// algorithm.set( KKT_TOLERANCE, 1e-6 );
	// algorithm.set( GLOBALIZATION_STRATEGY, GS_FULLSTEP );
	// algorithm.set( MAX_NUM_ITERATIONS, 1 );
	// algorithm.set( INFEASIBLE_QP_HANDLING, 1);

	algorithm.solve();


	return 0;
}



