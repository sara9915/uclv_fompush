#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <fstream>
#include <memory>
#include <cstdlib>
#include "helper.h"
#include "pushing.h"
#include <time.h>
#include <iomanip>
#include <stdio.h>
#include <math.h>
#include <complex>

#include <ctime>
#include <sys/time.h>
#include <sys/resource.h>

using namespace std;
using Eigen::MatrixXd;

Push::Push(char* Q_cost, char* Abar_str, char* Ain, char* bin, char* Aeq, char* beq): 
          env(), model(env), lhs(0) //Constructor of Push Class  //peterkty: put the initialization of member variable here
{
	Cbi.resize(3,3);
	Cci.resize(3,3);
	x_sensor.resize(7,1);
	delta_x_temp.resize(7,1);
	x_ref.resize(7,1);
	A_bar_temp.resize(7,7);
	gi.resize(2,1);
	gb.resize(2,1);
	theta_rel.resize(1,1);
	theta_g.resize(1,1);
	rc.resize(2,1);
	vc.resize(2,1);
	xc.resize(1,1);
	yc.resize(1,1);
	dxc.resize(1,1);
	dyc.resize(1,1);
	delta_u.resize(4,1);

	nu = 0.35;
	nu_pusher = 0.3;
	m = 1.0530;
	J = 0.00142155;
	a = 0.09;
	b = 0.09;
	A = a*b;
	g = 9.81;
	h = 1.f/1000; // Think about this!

	_Q_cost = Q_cost;
	_Ain = Ain;
	_bin = bin;
	_Aeq = Aeq;
	_beq = beq;
	_Abar_str = Abar_str;

	read_array();
	set_equation_sense();
	set_variable_type();
}

void Push::build_model()
{

	vars = model.addVars(lb, ub, NULL, vtype, NULL, num_variables);

	model.update();


	GRBQuadExpr obj = 0;
	for (int j = 0; j < num_variables; j++)
		obj += c[j]*vars[j];
	for (int i = 0; i < num_variables; i++)
		for (int j = 0; j < num_variables; j++)
			if (Q[i][j] != 0)
				obj += Q[i][j]*vars[i]*vars[j];

	model.setObjective(obj);

	stack_arrays();



	for (int i = 0; i < num_constraints; i++) {
		 lhs = 0;
		for (int j = 0; j < num_variables; j++)
			if (Atotal[i][j] != 0)
				lhs += Atotal[i][j]*vars[j];
		 constr[i] = model.addConstr(lhs, sense[i], btotal[i]);
	}

	model.update();
	model.getEnv().set(GRB_IntParam_OutputFlag,0);
};

void Push::update_model(MatrixXd qs, MatrixXd vs, MatrixXd qp, MatrixXd Target)
{

	q_slider  = qs;
	dq_slider = vs;
	q_pusher  = qp;
	theta     = q_slider(2);
	dtheta = dq_slider(2);

	Cbi << cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, 0, 0, 1;

	gi << Target(0) - q_slider(0), Target(1) - q_slider(1);
	gb = Cbi.topLeftCorner(2,2)*gi;

	complex<double> mycomplex (gb(0), gb(1));
	mycomplex = log(mycomplex);
	theta_rel << -mycomplex.imag();
	theta_g << theta - theta_rel(0);

	Cci << cos(theta_g(0)), sin(theta_g(0)), 0, -sin(theta_g(0)), cos(theta_g(0)), 0, 0, 0, 1;
	rc << Cci.topLeftCorner(2,2)*q_slider.topLeftCorner(2,1);
	xc << rc(0);
	yc << rc(1);

	vc << Cci.topLeftCorner(2,2)*dq_slider.topLeftCorner(2,1);
	dxc << vc(0);
	dyc << vc(1);
        
        //Find psi
        double xp = q_pusher(0);
        double yp = q_pusher(1);
        double x = q_slider(0);
        double y = q_slider(1);
        
        MatrixXd ripb(3,1);
        MatrixXd rbpb(3,1);
        
        ripb << xp-x,yp-y,0;
	rbpb = Cbi*ripb;
	// rbpb(0) = -0.045; //force value to be half length of square
        
        psi =rbpb(1);



//	x_sensor << q_slider, psi, dq_slider;

	x_ref    << 0,0,0,0,0.05,0,0;
	x_sensor << x_ref(0), x_ref(1), theta_rel, psi, dxc, dyc, dtheta;

	delta_x_temp = x_sensor -x_ref;
	delta_x_temp = delta_x_temp*1;
        
	for (int i=0;i<7;i++){
		for (int j=0;j<7;j++){
			A_bar_temp(i,j) = Abar[i][j]; }}
	b_temp = A_bar_temp*delta_x_temp;


	for (int i=0;i<7;i++){
		constr[num_ineq_constraints+i].set(GRB_DoubleAttr_RHS, b_temp(i)*1);
	}

	for (int i=0;i<7;i++){
		constr[num_ineq_constraints+i].set(GRB_DoubleAttr_RHS, b_temp(i)*1);
	}
};

double Push::optimize()
{
	model.optimize();

	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
		objval = model.get(GRB_DoubleAttr_ObjVal);
		for (int i = 0; i < NUM_VARIABLES; i++)
			solution[i] = vars[i].get(GRB_DoubleAttr_X);
		success = true;
	}
	for (int i=0;i<4;i++){delta_u(i,0) = solution[i];}
        // cout << " u " << solution<<endl;

return objval;

};

//**********************************************************************
void Push::read_file(FILE* myFile, int num_rows, int num_cols, double *A)
{
	int i;
	int j;
	float value;

	   for (i = 0; i < num_rows; i++)
	    {
		    const int RowOffset = (i * num_cols);
	    	for (j=0;j<num_cols;j++)
	        {;
	    		fscanf(myFile, "%e,",  &value);
	    		A[RowOffset + j] = (double)value;
	        }
	        }


}

//********************************************************************************
void Push::read_array()
{
	//	cout << Q_cost<<endl;
	const char* Q_File = _Q_cost;
	const char* Ain_File = _Ain;
	const char* bin_File = _bin;
	const char* Aeq_File = _Aeq;
	const char* beq_File = _beq;
	const char* Abar_File = _Abar_str;


	//	 Read H Matrix from txt file (from matlab code)
	myFile = fopen(Q_File, "r");
	read_file(myFile,  num_variables,  num_variables, Q[0]);
	fclose(myFile);


	//Read Abar Matrix from txt file (from matlab code)
	myFile = fopen(Abar_File, "r");
	read_file(myFile,  7,  7, Abar[0]);
	fclose(myFile);

	//Ain Matrices (for 3 modes)
	myFile = fopen(Ain_File, "r");
	read_file(myFile,  num_ineq_constraints,  num_variables, Ain[0]);
	fclose(myFile);

	//	     bin Matrices (for 3 modes)
	myFile = fopen(bin_File, "r");
	read_file(myFile,  num_ineq_constraints,  1, bin);
	fclose(myFile);

	// Aeq Matrices (for 3 modes)
	myFile = fopen(Aeq_File, "r");
	read_file(myFile,  num_eq_constraints,  num_variables, Aeq[0]);
	fclose(myFile);

	//beq Matrices (for 3 modes)
	myFile = fopen(beq_File, "r");
	read_file(myFile,  num_eq_constraints,  1, beq);
	fclose(myFile);

};

void Push::set_equation_sense()
{
	//	    read_array();
	for (int i=0; i<num_ineq_constraints; i++){

		sense[i] = '<';

	}
	for (int i=num_ineq_constraints; i<num_constraints; i++){
		sense[i] = '=';
	}
};

void Push::set_variable_type()
{
	// Set variable types (i.e. continuous 'C', or integer 'i') (Gurobi)
	for (int i=0; i<num_variables; i++){
		c[i] = 0;
		lb[i] = -10;
		ub[i] = 10;
		vtype[i]= 'C';
	}

}

//**********************************************************************
void Push::stack_arrays()
{
	for (int i=0; i<num_ineq_constraints; i++)
     {
		 btotal[i] = bin[i];
         for(int j = 0; j < num_variables; ++j)
         {
             Atotal[i][j] = Ain[i][j];
         }
     }

	int counter = 0;

	for (int i=num_ineq_constraints; i<num_ineq_constraints+num_eq_constraints; i++)
     {
		btotal[i] = beq[counter];

         for(int j = 0; j < num_variables; ++j)
         {
        	 Atotal[i][j] = Aeq[counter][j];
         }
         counter++;
     }
}




