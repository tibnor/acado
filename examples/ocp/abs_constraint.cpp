/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file   examples/ocp/abs_constraint.cpp
 *    \author Torstein Ingebrigtsen Bø
 *    \date   2009
 */

#include <acado_optimal_control.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>

int main() {

	USING_NAMESPACE_ACADO

	// INTRODUCE THE VARIABLES:
	// -------------------------

	DifferentialState	x;
	Control u[2];
	DifferentialEquation f;

	const double t_start = 0.0;
	const double t_end = 1.0;

	// DEFINE A DIFFERENTIAL EQUATION:
	// -------------------------------

	f << -dot(x) -x - u[0] - u[1];
	Expression power = 10*pow(abs(u[0]),1.5) + pow(abs(u[1]),1.5);

    GnuplotWindow window;
        window.addSubplot( x,"DifferentialState z" );
        window.addSubplot( u[0],"Control u[0]" );
        window.addSubplot( u[1],"Control u[1]" );
        window.addSubplot( power,"Power" );

	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	OCP ocp( t_start, t_end, 20 );

	ocp.minimizeMayerTerm ( x*x+.001*u[0]*u[0]+u[1]*u[1] );
	ocp.subjectTo( f );
	ocp.subjectTo( AT_START, x == 1.0 );
	ocp.subjectTo( -.5 <= u[0] <= .5 );
	ocp.subjectTo( -1 <= u[1] <= 1 );
	ocp.subjectTo( power <=  .5);


	// DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
	// ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp);

//    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    algorithm << window;
	algorithm.solve();

	return 0;
}

