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
 *    \file src/code_generation/integrators/irk_lifted_symmetric_export.cpp
 *    \author Rien Quirynen
 *    \date 2015
 */

#include <acado/code_generation/integrators/irk_export.hpp>
#include <acado/code_generation/integrators/irk_lifted_symmetric_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

SymmetricLiftedIRKExport::SymmetricLiftedIRKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ForwardLiftedIRKExport( _userInteraction,_commonHeaderName )
{
}

SymmetricLiftedIRKExport::SymmetricLiftedIRKExport( const SymmetricLiftedIRKExport& arg ) : ForwardLiftedIRKExport( arg )
{
}


SymmetricLiftedIRKExport::~SymmetricLiftedIRKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


SymmetricLiftedIRKExport& SymmetricLiftedIRKExport::operator=( const SymmetricLiftedIRKExport& arg ){

    if( this != &arg ){

    	ForwardLiftedIRKExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


ExportVariable SymmetricLiftedIRKExport::getAuxVariable() const
{
	ExportVariable max;
	if( NX1 > 0 ) {
		max = lin_input.getGlobalExportVariable();
	}
	if( NX2 > 0 || NXA > 0 ) {
		if( rhs.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = rhs.getGlobalExportVariable();
		}
		if( diffs_rhs.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_rhs.getGlobalExportVariable();
		}
		if( diffs_sweep.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_sweep.getGlobalExportVariable();
		}
		if( forward_sweep.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = forward_sweep.getGlobalExportVariable();
		}
		if( adjoint_sweep.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = adjoint_sweep.getGlobalExportVariable();
		}
	}
	if( NX3 > 0 ) {
		if( rhs3.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = rhs3.getGlobalExportVariable();
		}
		if( diffs_rhs3.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_rhs3.getGlobalExportVariable();
		}
	}
	uint i;
	for( i = 0; i < outputs.size(); i++ ) {
		if( outputs[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = outputs[i].getGlobalExportVariable();
		}
		if( diffs_outputs[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_outputs[i].getGlobalExportVariable();
		}
	}
	return max;
}


returnValue SymmetricLiftedIRKExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	ForwardLiftedIRKExport::getDataDeclarations( declarations, dataStruct );

	declarations.addDeclaration( rk_A_traj,dataStruct );
	declarations.addDeclaration( rk_S_traj,dataStruct );
	declarations.addDeclaration( rk_xxx_traj,dataStruct );

	declarations.addDeclaration( rk_b_trans,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	ForwardLiftedIRKExport::getFunctionDeclarations( declarations );

    return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::setDifferentialEquation(	const Expression& rhs_ )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if( rhs_.getDim() > 0 ) {
		OnlineData        dummy0;
		Control           dummy1;
		DifferentialState dummy2;
		AlgebraicState 	  dummy3;
		DifferentialStateDerivative dummy4;
		dummy0.clearStaticCounters();
		dummy1.clearStaticCounters();
		dummy2.clearStaticCounters();
		dummy3.clearStaticCounters();
		dummy4.clearStaticCounters();

		NX2 = rhs_.getDim() - NXA;
		x = DifferentialState("", NX1+NX2, 1);
		z = AlgebraicState("", NXA, 1);
		u = Control("", NU, 1);
		od = OnlineData("", NOD, 1);

		DifferentialEquation f;
		f << rhs_;

		NDX2 = f.getNDX();
		if( NDX2 > 0 && (NDX2 < NX2 || NDX2 > (NX1+NX2)) ) {
			return ACADOERROR( RET_INVALID_OPTION );
		}
		else if( NDX2 > 0 ) NDX2 = NX1+NX2;
		dx = DifferentialStateDerivative("", NDX2, 1);

		DifferentialEquation g;
		for( uint i = 0; i < rhs_.getDim(); i++ ) {
			g << forwardDerivative( rhs_(i), x );
			g << forwardDerivative( rhs_(i), z );
			g << forwardDerivative( rhs_(i), u );
			g << forwardDerivative( rhs_(i), dx );
		}

		DifferentialState sX("", NX,NX+NU);
		Expression Gx = sX.getCols(0,NX);
		Expression Gu = sX.getCols(NX,NX+NU);
		DifferentialEquation forward;
		for( uint i = 0; i < rhs_.getDim(); i++ ) {
			// NOT YET IMPLEMENTED FOR DAES OR IMPLICIT ODES
			if( NDX2 > 0 || NXA > 0 ) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);
			forward << multipleForwardDerivative( rhs_(i), x, Gx );
			forward << multipleForwardDerivative( rhs_(i), x, Gu ) + forwardDerivative( rhs_(i), u );
		}

		DifferentialState lambda("", NX,1);
		DifferentialEquation backward;
		backward << backwardDerivative( rhs_, x, lambda );

		DifferentialEquation h;
		if( (ExportSensitivityType)sensGen == INEXACT ) {
			DifferentialState sX("", NX,NX+NU);
			DifferentialState dKX("", NDX2,NX+NU);
			DifferentialState dKZ("", NXA,NX+NU);

			Expression tmp = zeros<double>(NX+NXA,NX);
			tmp.appendCols(forwardDerivative( rhs_, u ));

			if(NXA > 0 && NDX2 == 0) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);

			// forward sweep
			if(NXA == 0 && NDX2 == 0) {
				h << multipleForwardDerivative( rhs_, x, sX ) - tmp;
			}
			else {
				Expression tmp2 = tmp + multipleForwardDerivative( rhs_, dx, dKX );
				Expression tmp3 = tmp2 + multipleForwardDerivative( rhs_, z, dKZ );
				h << multipleForwardDerivative( rhs_, x, sX ) - tmp3;
			}

		}

		if( f.getNT() > 0 ) timeDependant = true;

		return (rhs.init( f,"acado_rhs",NX,NXA,NU,NP,NDX,NOD ) &
				diffs_rhs.init( g,"acado_diffs",NX,NXA,NU,NP,NDX,NOD ) &
				forward_sweep.init( forward,"acado_forward",NX*(2+NX+NU),NXA,NU,NP,NDX,NOD ) &
				adjoint_sweep.init( backward,"acado_backward",NX*(2+NX+NU),NXA,NU,NP,NDX,NOD ) &
				diffs_sweep.init( h,"acado_diff_sweep",NX+(NX+NDX2+NXA)*(NX+NU),NXA,NU,NP,NDX,NOD ) );
	}
	return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::getCode(	ExportStatementBlock& code )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	int mode;
	get( IMPLICIT_INTEGRATOR_MODE, mode );
	int liftMode;
	get( LIFTED_INTEGRATOR_MODE, liftMode );
	if ( (ExportSensitivityType)sensGen != SYMMETRIC ) ACADOERROR( RET_INVALID_OPTION );
	if( (ImplicitIntegratorMode)mode != LIFTED ) ACADOERROR( RET_INVALID_OPTION );
	if( liftMode != 1 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if( (ExportSensitivityType)sensGen == INEXACT ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if( NXA > 0) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( CONTINUOUS_OUTPUT || NX1 > 0 || NX3 > 0 || !equidistantControlGrid() ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( NX1 > 0 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( exportRhs ) {
		if( NX2 > 0 || NXA > 0 ) {
			code.addFunction( rhs );
			code.addStatement( "\n\n" );
			code.addFunction( diffs_rhs );
			code.addStatement( "\n\n" );
			code.addFunction( diffs_sweep );
			code.addStatement( "\n\n" );
			code.addFunction( forward_sweep );
			code.addStatement( "\n\n" );
			code.addFunction( adjoint_sweep );
			code.addStatement( "\n\n" );
		}

		if( NX3 > 0 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

		if( CONTINUOUS_OUTPUT ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}
	if( NX2 > 0 || NXA > 0 ) solver->getCode( code );
	code.addLinebreak(2);

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	// export RK scheme
	uint run5;
	std::string tempString;
	
	initializeDDMatrix();
	initializeCoefficients();

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	DMatrix tmp = AA;
	ExportVariable Ah( "Ah_mat", tmp*=h, STATIC_CONST_REAL );
	code.addDeclaration( Ah );
	code.addLinebreak( 2 );
	// TODO: Ask Milan why this does NOT work properly !!
	Ah = ExportVariable( "Ah_mat", numStages, numStages, STATIC_CONST_REAL, ACADO_LOCAL );

	DVector BB( bb );
	ExportVariable Bh( "Bh_mat", DMatrix( BB*=h ) );

	DVector CC( cc );
	ExportVariable C;
	if( timeDependant ) {
		C = ExportVariable( "C_mat", DMatrix( CC*=(1.0/grid.getNumIntervals()) ), STATIC_CONST_REAL );
		code.addDeclaration( C );
		code.addLinebreak( 2 );
		C = ExportVariable( "C_mat", 1, numStages, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	code.addComment(std::string("Fixed step size:") + toString(h));

	ExportVariable determinant( "det", 1, 1, REAL, ACADO_LOCAL, true );
	integrate.addDeclaration( determinant );

	ExportIndex i( "i" );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	ExportIndex run( "run" );
	ExportIndex run1( "run1" );
	ExportIndex tmp_index1("tmp_index1");
	ExportIndex tmp_index2("tmp_index2");
	ExportIndex tmp_index3("tmp_index3");
	ExportIndex tmp_index4("tmp_index4");
	ExportIndex k_index("k_index");
	ExportIndex shooting_index("shoot_index");
	ExportVariable tmp_meas("tmp_meas", 1, outputGrids.size(), INT, ACADO_LOCAL);

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONST_INT );
		code.addDeclaration( numStepsV );
		code.addLinebreak( 2 );
		integrate.addStatement( std::string( "int " ) + numInt.getName() + " = " + numStepsV.getName() + "[" + rk_index.getName() + "];\n" );
	}

	prepareOutputEvaluation( code );

	integrate.addIndex( i );
	integrate.addIndex( j );
	integrate.addIndex( k );
	integrate.addIndex( run );
	integrate.addIndex( run1 );
	integrate.addIndex( tmp_index1 );
	integrate.addIndex( tmp_index2 );
	integrate.addIndex( tmp_index3 );
	integrate.addIndex( shooting_index );
	integrate.addIndex( k_index );
	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !equidistantControlGrid()) ) {
		integrate.addIndex( tmp_index4 );
	}
	integrate << shooting_index.getFullName() << " = " << rk_index.getFullName() << ";\n";
	integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );
	if( (inputDim-diffsDim) > NX+NXA ) {
		integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
		integrate.addStatement( rk_seed.getCols( 2*NX+NX*(NX+NU),NX+NX*(NX+NU)+inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	}
	integrate.addLinebreak( );
	if( liftMode == 1 || (liftMode == 4 && (ExportSensitivityType)sensGen == INEXACT) ) {
		integrate.addStatement( rk_delta.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) - rk_Xprev.getRow(shooting_index) );
		integrate.addStatement( rk_Xprev.getRow(shooting_index) == rk_eta.getCols( 0,NX ) );

		integrate.addStatement( rk_delta.getCols( NX,NX+NU ) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) - rk_Uprev.getRow(shooting_index) );
		integrate.addStatement( rk_Uprev.getRow(shooting_index) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) );
	}

    // integrator FORWARD loop:
	integrate.addComment("------------ Forward loop ------------:");
	ExportForLoop tmpLoop( run, 0, grid.getNumIntervals() );
	ExportStatementBlock *loop;
	if( equidistantControlGrid() ) {
		loop = &tmpLoop;
	}
	else {
	    loop = &integrate;
		loop->addStatement( std::string("for(") + run.getName() + " = 0; " + run.getName() + " < " + numInt.getName() + "; " + run.getName() + "++ ) {\n" );
	}

//	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		// Set rk_diffsPrev:
		loop->addStatement( std::string("if( run > 0 ) {\n") );
		if( NX1 > 0 ) {
			ExportForLoop loopTemp1( i,0,NX1 );
			loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,0,NX1 ) == rk_eta.getCols( i*NX+NX+NXA,i*NX+NX+NXA+NX1 ) );
			if( NU > 0 ) loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,NX1,NX1+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1),i*NU+(NX+NXA)*(NX+1)+NU ) );
			loop->addStatement( loopTemp1 );
		}
		if( NX2 > 0 ) {
			ExportForLoop loopTemp2( i,0,NX2 );
			loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,0,NX1+NX2 ) == rk_eta.getCols( i*NX+NX+NXA+NX1*NX,i*NX+NX+NXA+NX1*NX+NX1+NX2 ) );
			if( NU > 0 ) loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,NX1+NX2,NX1+NX2+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+NX1*NU,i*NU+(NX+NXA)*(NX+1)+NX1*NU+NU ) );
			loop->addStatement( loopTemp2 );
		}
		if( NX3 > 0 ) {
			ExportForLoop loopTemp3( i,0,NX3 );
			loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,0,NX ) == rk_eta.getCols( i*NX+NX+NXA+(NX1+NX2)*NX,i*NX+NX+NXA+(NX1+NX2)*NX+NX ) );
			if( NU > 0 ) loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,NX,NX+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU,i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU+NU ) );
			loop->addStatement( loopTemp3 );
		}
		loop->addStatement( std::string("}\nelse{\n") );
		DMatrix eyeM = eye<double>(NX);
		eyeM.appendCols(zeros<double>(NX,NU));
		loop->addStatement( rk_diffsPrev2 == eyeM );
		loop->addStatement( std::string("}\n") );
//	}

	// SAVE rk_diffsPrev2 in the rk_S_traj variable:
	loop->addStatement( rk_S_traj.getRows(run*NX,(run+1)*NX) == rk_diffsPrev2 );

	loop->addStatement( k_index == (shooting_index*grid.getNumIntervals()+run)*(NX+NXA) );

	// FIRST update using term from optimization variables:
	if( liftMode == 1 || (liftMode == 4 && (ExportSensitivityType)sensGen == INEXACT) ) {
		ExportForLoop loopTemp1( i,0,NX+NXA );
		loopTemp1.addStatement( j == k_index+i );
		loopTemp1.addStatement( tmp_index1 == j*(NX+NU) );
		ExportForLoop loopTemp2( run1,0,numStages );
		loopTemp2.addStatement( rk_kkk.getElement( j,run1 ) += rk_delta*rk_diffK.getSubMatrix( tmp_index1,tmp_index1+NX+NU,run1,run1+1 ) );
		loopTemp1.addStatement( loopTemp2 );
		loop->addStatement( loopTemp1 );
	}

	// Evaluate all stage values for reuse:
	evaluateAllStatesImplicitSystem( loop, k_index, Ah, C, run1, j, tmp_index1 );

	// SAVE rk_stageValues in the rk_xxx_traj variable:
	loop->addStatement( rk_xxx_traj.getCols(run*numStages*(NX2+NXA),(run+1)*numStages*(NX2+NXA)) == rk_stageValues );

	solveImplicitSystem( loop, i, run1, j, tmp_index1, k_index, Ah, C, determinant, true );

	// SAVE rk_A in the rk_A_traj variable:
	loop->addStatement( rk_A_traj.getRows(run*numStages*(NX2+NXA),(run+1)*numStages*(NX2+NXA)) == rk_A );

	// Evaluate sensitivities:
	// !! NEW !! Let us propagate the forward sensitivities as in a VDE system
	loop->addStatement( rk_seed.getCols(NX,NX+NX*(NX+NU)) == rk_diffsPrev2.makeRowVector() );
	ExportForLoop loop_sens( i,0,numStages );
	loop_sens.addStatement( rk_seed.getCols(0,NX) == rk_stageValues.getCols(i*(NX+NXA),i*(NX+NXA)+NX) );
	loop_sens.addFunctionCall( forward_sweep.getName(), rk_seed, rk_diffsTemp2.getAddress(i,0) );
	loop->addStatement( loop_sens );

	evaluateRhsSensitivities( loop, run1, i, j, tmp_index1, tmp_index2 );
	allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Bh, false );

	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
	// update rk_kkk:
	ExportForLoop loopTemp( j,0,numStages );
	for( run5 = 0; run5 < NX2; run5++ ) {
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX1+run5,j ) += rk_b.getElement( j*(NX2+NXA)+run5,0 ) );		// differential states
		}
		else {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX1+run5,j ) += rk_b.getElement( j*NX2+run5,0 ) );			// differential states
		}
	}
	for( run5 = 0; run5 < NXA; run5++ ) {
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX+run5,j ) += rk_b.getElement( j*(NX2+NXA)+NX2+run5,0 ) );		// algebraic states
		}
		else {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX+run5,j ) += rk_b.getElement( numStages*NX2+j*NXA+run5,0 ) );	// algebraic states
		}
	}
	loop->addStatement( loopTemp );

	// update rk_eta:
	for( run5 = 0; run5 < NX; run5++ ) {
		loop->addStatement( rk_eta.getCol( run5 ) += rk_kkk.getRow( k_index+run5 )*Bh );
	}
	if( NXA > 0) {
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("if( run == 0 ) {\n") );
		}
		for( run5 = 0; run5 < NXA; run5++ ) {
			loop->addStatement( rk_eta.getCol( NX+run5 ) == rk_kkk.getRow( k_index+NX+run5 )*tempCoefs );
		}
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("}\n") );
		}
	}


	// Computation of the sensitivities using the CHAIN RULE:
	updateImplicitSystem(loop, i, j, tmp_index2);

//	loop->addStatement( std::string( reset_int.get(0,0) ) + " = 0;\n" );

	loop->addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );

    // end of the forward integrator loop.
    if( !equidistantControlGrid() ) {
		loop->addStatement( "}\n" );
	}
    else {
    	integrate.addStatement( *loop );
    }

    // integrator BACKWARD loop:
	integrate.addComment("------------ BACKWARD loop ------------:");
	ExportForLoop tmpLoop2( run, grid.getNumIntervals()-1, -1, -1 );
	ExportStatementBlock *loop2;
	if( equidistantControlGrid() ) {
		loop2 = &tmpLoop2;
	}
	else {
	    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}

	// Compute \hat{lambda}:
	// vec(j*NX+1:j*NX+NX) = -Bh_vec(j+1)*dir_tmp;
	for( run5 = 0; run5 < numStages; run5++ ) {
		DMatrix zeroV = zeros<double>(1,NX);
		loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) == zeroV );
		loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) -= Bh.getRow(run5)*rk_eta.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) );
	}
	loop2->addFunctionCall( solver->getNameSolveTransposeReuseFunction(),rk_A_traj.getAddress(run*numStages*(NX2+NXA),0),rk_b_trans.getAddress(0,0),rk_auxSolver.getAddress(0,0) );

	for( run5 = 0; run5 < numStages; run5++ ) {
		loop2->addStatement( rk_seed.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
		loop2->addStatement( rk_seed.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) == rk_b_trans.getCols(run5*NX,(run5+1)*NX) );
		loop2->addFunctionCall( adjoint_sweep.getName(), rk_seed, rk_diffsTemp2.getAddress(0,0) );
		loop2->addStatement( rk_eta.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) += rk_diffsTemp2.getSubMatrix(0,1,0,NX) );
	}

	loop2->addStatement( rk_ttt -= DMatrix(1.0/grid.getNumIntervals()) );
    // end of the backward integrator loop.
    if( !equidistantControlGrid() ) {
    	return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}
    else {
    	integrate.addStatement( *loop2 );
    }

    integrate.addStatement( std::string( "if( " ) + determinant.getFullName() + " < 1e-12 ) {\n" );
    integrate.addStatement( error_code == 2 );
    integrate.addStatement( std::string( "} else if( " ) + determinant.getFullName() + " < 1e-6 ) {\n" );
    integrate.addStatement( error_code == 1 );
    integrate.addStatement( std::string( "} else {\n" ) );
    integrate.addStatement( error_code == 0 );
    integrate.addStatement( std::string( "}\n" ) );

	code.addFunction( integrate );
    code.addLinebreak( 2 );

    return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::evaluateRhsSensitivities( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2 )
{
	if( NX2 > 0 ) {
		ExportForLoop loop1( index2,0,numStages );
		ExportForLoop loop2( index3,0,NX2+NXA );
		loop2.addStatement( tmp_index1 == index2*(NX2+NXA)+index3 );
		ExportForLoop loop3( index1,0,NX2 );
		loop3.addStatement( tmp_index2 == index1+index3*(NVARS2) );
		loop3.addStatement( rk_b.getElement( tmp_index1,1+index1 ) == 0.0 - rk_diffsTemp2.getElement( index2,tmp_index2 ) );
		loop2.addStatement( loop3 );

		ExportForLoop loop4( index1,0,NU );
		loop4.addStatement( tmp_index2 == index1+index3*(NVARS2)+NX1+NX2+NXA );
		loop4.addStatement( rk_b.getElement( tmp_index1,1+NX+index1 ) == 0.0 - rk_diffsTemp2.getElement( index2,tmp_index2 ) );
		loop2.addStatement( loop4 );
		loop1.addStatement( loop2 );
		block->addStatement( loop1 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::allSensitivitiesImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportIndex& tmp_index3, const ExportIndex& k_index, const ExportVariable& Bh, bool update )
{
	if( NX2 > 0 ) {
		int linSolver;
		get( LINEAR_ALGEBRA_SOLVER, linSolver );
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );  // We compute the algebraic variables at the beginning of the shooting interval !

		// call the linear solver:
		if( NDX2 > 0 ) {
			block->addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_I.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		}
		else {
			block->addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		}

		// update rk_diffK with the new sensitivities:
		ExportForLoop loop20( index2,0,numStages );
		ExportForLoop loop21( index3,0,NX2 );
		loop21.addStatement( tmp_index1 == (k_index + NX1 + index3)*(NX+NU) );
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			loop21.addStatement( tmp_index3 == index2*(NX2+NXA)+index3 );
		}
		else {
			loop21.addStatement( tmp_index3 == index2*NX2+index3 );
		}
		ExportForLoop loop22( index1,0,NX2 );
		loop22.addStatement( tmp_index2 == tmp_index1+index1 );
		if( update ) {
			loop22.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+index1) );
		}
		else {
			loop22.addStatement( rk_diffK.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+index1) );
		}
		loop21.addStatement( loop22 );

		ExportForLoop loop23( index1,0,NU );
		loop23.addStatement( tmp_index2 == tmp_index1+NX+index1 );
		if( update ) {
			loop23.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+NX+index1) );
		}
		else {
			loop23.addStatement( rk_diffK.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+NX+index1) );
		}
		loop21.addStatement( loop23 );
		loop20.addStatement( loop21 );
		if( NXA > 0 ) {
			ExportForLoop loop24( index3,0,NXA );
			loop24.addStatement( tmp_index1 == (k_index + NX + index3)*(NX+NU) );
			if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
				loop24.addStatement( tmp_index3 == index2*(NX2+NXA)+NX2+index3 );
			}
			else {
				loop24.addStatement( tmp_index3 == numStages*NX2+index2*NXA+index3 );
			}
			ExportForLoop loop25( index1,0,NX2 );
			loop25.addStatement( tmp_index2 == tmp_index1+index1 );
			if( update ) {
				loop25.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+index1) );
			}
			else {
				loop25.addStatement( rk_diffK.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+index1) );
			}
			loop24.addStatement( loop25 );

			ExportForLoop loop26( index1,0,NU );
			loop26.addStatement( tmp_index2 == tmp_index1+NX+index1 );
			if( update ) {
				loop26.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+NX+index1) );
			}
			else {
				loop26.addStatement( rk_diffK.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+NX+index1) );
			}
			loop24.addStatement( loop26 );
			loop20.addStatement( loop24 );
		}
		block->addStatement( loop20 );

		// update rk_diffsNew with the new sensitivities:
		ExportForLoop loop3( index2,0,NX2 );
		loop3.addStatement( tmp_index1 == (k_index + NX1 + index2)*(NX+NU) );
		ExportForLoop loop31( index1,0,NX2 );
		loop31.addStatement( tmp_index2 == tmp_index1+index1 );
		loop31.addStatement( rk_diffsNew2.getElement( index2,index1 ) == rk_diffsPrev2.getElement( index2,index1 ) );
		loop31.addStatement( rk_diffsNew2.getElement( index2,index1 ) += rk_diffK.getRow( tmp_index2 )*Bh );
		loop3.addStatement( loop31 );

		ExportForLoop loop32( index1,0,NU );
		loop32.addStatement( tmp_index2 == tmp_index1+NX+index1 );
		loop32.addStatement( rk_diffsNew2.getElement( index2,NX+index1 ) == rk_diffsPrev2.getElement( index2,NX+index1 ) );
		loop32.addStatement( rk_diffsNew2.getElement( index2,NX+index1 ) += rk_diffK.getRow( tmp_index2 )*Bh );
		loop3.addStatement( loop32 );
		block->addStatement( loop3 );
		if( NXA > 0 ) {
			ACADOERROR( RET_NOT_IMPLEMENTED_YET );
//			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
//				block->addStatement( std::string("if( run == 0 ) {\n") );
//			}
//			ExportForLoop loop4( index2,0,NXA );
//			loop4.addStatement( tmp_index1 == (k_index + NX + index2)*(NX+NU) );
//			ExportForLoop loop41( index1,0,NX2 );
//			loop41.addStatement( tmp_index2 == tmp_index1+index1 );
//			loop41.addStatement( rk_diffsNew2.getElement( index2+NX2,index1 ) == rk_diffK.getRow( tmp_index2 )*tempCoefs );
//			loop4.addStatement( loop41 );
//
//			ExportForLoop loop42( index1,0,NU );
//			loop42.addStatement( tmp_index2 == tmp_index1+NX+index1 );
//			loop42.addStatement( rk_diffsNew2.getElement( index2+NX2,NX+index1 ) == rk_diffK.getRow( tmp_index2 )*tempCoefs );
//			loop4.addStatement( loop42 );
//			block->addStatement( loop4 );
//			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
//				block->addStatement( std::string("}\n") );
//			}
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::setup( )
{
	ForwardLiftedIRKExport::setup();

	uint numX = NX*(NX+1)/2.0;
	uint numU = NU*(NU+1)/2.0;
	diffsDim   = NX + NX*(NX+NU) + numX + NX*NU + numU;
	inputDim = NX + diffsDim + NU + NOD;

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;

	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );

	rk_b_trans = ExportVariable( "rk_b_trans", 1, numStages*(NX+NXA), REAL, structWspace );

	rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+NX+NU+NOD+timeDep, REAL, structWspace );
	rk_Xprev = ExportVariable( "rk_Xprev", N, NX, REAL, ACADO_VARIABLES );
	rk_A_traj = ExportVariable( "rk_A_traj", grid.getNumIntervals()*numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
	rk_xxx_traj = ExportVariable( "rk_stageV_traj", 1, grid.getNumIntervals()*numStages*(NX+NXA), REAL, structWspace );
	rk_S_traj = ExportVariable( "rk_S_traj", grid.getNumIntervals()*NX, NX+NU, REAL, structWspace );

    return SUCCESSFUL_RETURN;
}



// PROTECTED:


CLOSE_NAMESPACE_ACADO

// end of file.
