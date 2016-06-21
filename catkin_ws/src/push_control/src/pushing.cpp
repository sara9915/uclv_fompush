#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <fstream>
#include <memory>
#include <cstdlib>
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

	//GRBEnv env = GRBEnv();   // peterkty: no need of these
	//GRBModel model = GRBModel(env);
	//GRBVar* vars;
	//GRBLinExpr lhs = 0;
//	GRBConstr constr[NUM_CONSTRAINTS] ;
}

void Push::build_model()
{

	vars = model.addVars(lb, ub, NULL, vtype, NULL, num_variables);

	model.update();


	GRBQuadExpr obj = 0;
	//
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
//	c1[1].set(GRB_DoubleAttr_RHS, 2.0);
//	cout<<"Print Constraints"<<endl;
//	double e[] = model.getConstrs();

};

void Push::update_model(MatrixXd qs, MatrixXd vs, MatrixXd qp, MatrixXd Target)
{

	q_slider  = qs;
	dq_slider = vs;
	q_pusher  = qp;
	theta     = q_slider(2);
	dtheta = dq_slider(2);

//	cout<<log(mycomplex)<<endl;
//	cout<<log(mycomplex)<<endl;
//	theta_rel << mycomplex;

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



//	x_sensor << q_slider, psi, dq_slider;

	x_ref    << 0,0,0,0,0.05,0,0;
	x_sensor << x_ref(0), x_ref(1), theta_rel, psi, dxc, dyc, dtheta;

	delta_x_temp = x_sensor -x_ref;

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
//	for (int i=0;i<7;i++){
//		constr[num_ineq_constraints+i].set(GRB_DoubleAttr_RHS, b_temp(i)*1);
//	}
//.getEnv().Set(GRB_IntParam_OutputFlag,0);
	model.optimize();
//	GRBVar* vars = model.getVars();
////
//	cout << "********************************************"<<endl;
//	cout << "Objective Value:"<<endl;

	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
		objval = model.get(GRB_DoubleAttr_ObjVal);
		for (int i = 0; i < NUM_VARIABLES; i++)
			solution[i] = vars[i].get(GRB_DoubleAttr_X);
		success = true;
	}
//	cout<< objval<<endl;


	for (int i=0;i<4;i++){delta_u(i,0) = solution[i];}

//	delta_u(0,0) = solution[0];
//	cout << "********************************************"<<endl;

return objval;

};

MatrixXd Push::inverse_dynamics()
{

	MatrixXd M_inv(3,3);
	MatrixXd w_b(3,1);
	MatrixXd w_i(3,1);
	MatrixXd r_pb_b(3,1);
	MatrixXd r_pb_i(3,1);
	MatrixXd n(3,1);
	MatrixXd d1(3,1);
	MatrixXd d2(3,1);
	MatrixXd vp(2,1);

	M_inv << 1/m,0,0,0,1/m,0,0,0,1/J;


//
	double x = q_slider(0);
	double y = q_slider(1);
//	double theta = q_slider(2);

	double dx = dq_slider(0);
	double dy = dq_slider(1);
	double dtheta = dq_slider(2);

	double xp = q_pusher(0);
	double yp = q_pusher(1);

	w_b << 0,0,dtheta;
	r_pb_i << xp-x,yp-y,0;

	n << cos(theta), sin(theta), cos(theta)*(y-yp) - sin(theta)*(x-xp);
	d1 << -sin(theta), cos(theta), -cos(theta)*(x-xp) - sin(theta)*(y-yp);
	d2 << sin(theta), -cos(theta), cos(theta)*(x-xp) + sin(theta)*(y-yp);

//	cout<<  dq_slider<<endl;

	w_i = Cbi.transpose()*w_b;
	r_pb_b = Cbi*r_pb_i;
	double psi =r_pb_b(2);

//	cout <<Cbi<<endl;
////
////
//	Cbi << cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, 0, 0, 1;
//
//
	MatrixXd f_f(3,1);
	MatrixXd Temp(3,2);
	MatrixXd v_i(2,1);
	MatrixXd v_i_3d(3,1);
	MatrixXd n3(3,1);
	MatrixXd n_f(3,1);
//
	n3<<0,0,1;
	v_i << dx,dy;
	v_i_3d << dx,dy,0;
	Temp << 1,0,0,1,0,0;
//
	// Gaussian points
	double ag = -a/2;
	double bg = a/2;
	double cg = -b/2;
	double dg = b/2;

	double h1 = (bg-ag)/2;
	double h2 = (bg+ag)/2;
	double h3 = (dg-cg)/2;
	double h4 = (dg+cg)/2;
//
	double w1g  = 1;
	double w2g = 1;
//
	double x1g = sqrt(1/3);
	double x2g = -sqrt(1/3);

	MatrixXd value1(1,1);
	MatrixXd value2(1,1);
	MatrixXd value3(1,1);
	MatrixXd value4(1,1);
	MatrixXd point1(3,1);
	MatrixXd point2(3,1);
	MatrixXd point3(3,1);
	MatrixXd point4(3,1);
	MatrixXd v_point1(3,1);
	MatrixXd v_point2(3,1);
	MatrixXd v_point3(3,1);
	MatrixXd v_point4(3,1);
	MatrixXd v_point1_norm(3,1);
	MatrixXd v_point2_norm(3,1);
	MatrixXd v_point3_norm(3,1);
	MatrixXd v_point4_norm(3,1);
//
	point1 <<h1*x1g + h2, h1*x1g+h2, 0;
	point2 <<h1*x1g+h2, h1*x2g+h2, 0;
	point3 <<h1*x2g+h2, h1*x1g+h2, 0;
	point4 <<h1*x2g+h2, h1*x2g+h2, 0;
//
	v_point1 = Cbi*v_i_3d + cross_op(w_b)*point1;
	v_point2 = Cbi*v_i_3d + cross_op(w_b)*point1;
	v_point3 = Cbi*v_i_3d + cross_op(w_b)*point1;
	v_point4 = Cbi*v_i_3d + cross_op(w_b)*point1;

	v_point1_norm = v_point1/v_point1.norm();
	v_point2_norm = v_point2/v_point2.norm();
	v_point3_norm = v_point3/v_point3.norm();
	v_point4_norm = v_point4/v_point4.norm();

	value1 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point1)*v_point1_norm)) ;
	value2 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point2)*v_point2_norm)) ;
	value3 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point3)*v_point3_norm)) ;
	value4 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point4)*v_point4_norm)) ;
//
	MatrixXd integral(1,1);
//
	integral = h1*h3 *( w1g*w1g*value1 + w1g*w2g*value2 + w2g*w1g*value3 + w2g*w2g* value4);
//
//
	if (v_i.norm()  < 0.001)
	{
		f_f = MatrixXd::Zero(3,1);
	}
	else{
		f_f = -Temp*nu*m*g*v_i/v_i.norm() ;
	}
//
//
	if (dtheta < 0.001){
		n_f << 0,0,0;
	}
	else
	{
		n_f <<0,0, integral(0);
	}

	MatrixXd f_friction(3,1);

	f_friction = f_f + n_f;

//

	MatrixXd dq_slider_next(3,1);
	MatrixXd w_b_next(1,1);
	MatrixXd Rotational_term(3,1);
	MatrixXd dr_pb_i(3,1);
	MatrixXd dpsi_vec(3,1);
	MatrixXd vc(3,1);
	MatrixXd u(4,1);

	for (int i=0;i<4;i++)
	{
	u(i) = solution[i];
	}
	double cn   = u(0) + 3.6155;
	double beta1= u(1);
	double beta2= u(2);
	double dpsi = u(3);
//
	dq_slider_next = dq_slider + h*M_inv*(f_friction + n*cn + d1*beta1 + d2*beta2 );

	w_b_next = n3*dq_slider_next(2);
	Rotational_term = cross_op(w_b_next)*r_pb_i;
	dpsi_vec << 0, dpsi, 0;
	dr_pb_i = Cbi.transpose()*dpsi_vec;
	vc =  dq_slider_next + Rotational_term + dr_pb_i;
//
//
	vp(0) = vc(0);
	vp(1) = vc(1);


	return vp;
}

//*************************************************************************

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

//**********************************************************************

//**********************************************************************
//
//void Push::stack_A_arrays(double *A, double *Ain, double *Aeq, int num_ineq_constraints, int num_eq_constraints, int num_variables)
//{
//
//	for (int i=0; i<num_ineq_constraints; i++)
//     {
//		 const int RowOffset_in = (i * num_variables);
//         const int RowOffset = (i * num_variables);
//         for(int j = 0; j < num_variables; ++j)
//         {
//             A[RowOffset + j] = Ain[RowOffset + j];
//         }
//     }
//
//
//	int counter = 0;
//
//	for (int i=num_ineq_constraints; i<num_ineq_constraints+num_eq_constraints; i++)
//     {
//		 const int RowOffset_eq = (counter * num_variables);
//         const int RowOffset = (i * num_variables);
//
//         for(int j = 0; j < num_variables; ++j)
//         {
//             A[RowOffset + j] = Aeq[RowOffset_eq + j];
//         }
//         counter++;
//     }
//
//
//
//
//}
//
////**********************************************************************
//
//void Push::stack_b_arrays(double *b, double *bin, double *beq, int num_ineq_constraints, int num_eq_constraints, int num_variables)
//{
//	for (int i=0; i<num_ineq_constraints; i++)
//     {
//             b[i] = bin[i];
//     }
//
//	int counter = 0;
//
//	for (int i=num_ineq_constraints; i<num_ineq_constraints+num_eq_constraints; i++)
//     {
//             b[i] = beq[counter];
//         counter++;
//     }
//
//
//}

//**********************************************************************
//void print_square_array(double *A, int rows, int cols)
//{
//
//	for (int i=0; i<rows; i++)
//     {
//         const int RowOffset = (i * cols);
//         for(int j = 0; j < cols; ++j)
//         {
//        	 if (j==0) {cout << "[";}
//             cout << setw(3) << A[RowOffset + j]<< " ";
//             if (j==cols-1) {cout << "]";}
//         }
//         cout << endl;
//     }
//	cout << endl;
//}
//
////**********************************************************************
//void print_array(double *A, int rows)
//{
//
//	for (int i=0; i<rows; i++)
//     {
//             cout <<  "[" <<setw(4) << A[i] << setw(1) <<"]";
//             cout << endl;
//     }
//	cout << endl;
//}

//**********************************************************************
/*void read_array1(FILE* myFile, int num_rows, int num_cols, double *A)
{
	int i;
	int j;
	float value;



	   for (i = 0; i < num_rows; i++)
	    {
		    const int RowOffset = (i * num_cols);
	    	for (j=0;j<num_cols;j++)
	        {;
	    		//fscanf(myFile, "%d,")<<endl;
	    		fscanf(myFile, "%e,",  &value);
	    		A[RowOffset + j] = (double)value;
	        }
	        }


}*/

//********************************************************************************

MatrixXd inverse_dynamics2(MatrixXd q_pusher, MatrixXd q_slider, MatrixXd dq_slider, MatrixXd u, double xp_des, double yp_des)
{
//
	const double nu = 0.35;
	const double nu_pusher = 0.3;
	const double m = 1.0530;
	const double J = 0.00142155;
	const double a = 0.09;
	const double b = 0.09;
	const double A = a*b;
	const double g = 9.81;
	const double h = 1.f/1000.0; // Think about this!

	MatrixXd M_inv(3,3);
	MatrixXd Cbi(3,3);
        MatrixXd Cbi_T(3,3);
	MatrixXd w_b(3,1);
	MatrixXd w_i(3,1);
	MatrixXd r_pb_b(3,1);
	MatrixXd r_pb_i(3,1);
	MatrixXd n(3,1);
	MatrixXd d1(3,1);
	MatrixXd d2(3,1);
	MatrixXd vp(2,1);

	M_inv << 1/m,0,0,0,1/m,0,0,0,1/J;


//
	double x = q_slider(0);
	double y = q_slider(1);
	double theta = q_slider(2)*1;

	double dx = dq_slider(0);
	double dy = dq_slider(1);
	double dtheta = dq_slider(2);

	double xp = q_pusher(0);
	double yp = q_pusher(1);

	w_b << 0,0,dtheta;
	r_pb_i << xp-x,yp-y,0;
        
	n << cos(theta), sin(theta), cos(theta)*(y-yp) - sin(theta)*(x-xp);
	d1 << -sin(theta), cos(theta), -cos(theta)*(x-xp) - sin(theta)*(y-yp);
	d2 << sin(theta), -cos(theta), cos(theta)*(x-xp) + sin(theta)*(y-yp);

	w_i = Cbi.transpose()*w_b;
	r_pb_b = Cbi*r_pb_i;
	double psi =r_pb_b(2);
//
//
// theta=0;
	Cbi << cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, 0, 0, 1;
        Cbi_T = Cbi.transpose();
	MatrixXd f_f(3,1);
	MatrixXd Temp(3,2);
	MatrixXd v_i(2,1);
	MatrixXd v_i_3d(3,1);
	MatrixXd n3(3,1);
	MatrixXd n_f(3,1);

	n3<<0,0,1;
	v_i << dx,dy;
	v_i_3d << dx,dy,0;
	Temp << 1,0,0,1,0,0;

	// Gaussian points
	double ag = -a/2;
	double bg = a/2;
	double cg = -b/2;
	double dg = b/2;

	double h1 = (bg-ag)/2;
	double h2 = (bg+ag)/2;
	double h3 = (dg-cg)/2;
	double h4 = (dg+cg)/2;

	double w1g = 1;
	double w2g = 1;

	double x1g = sqrt(1/3);
	double x2g = -sqrt(1/3);

	MatrixXd value1(1,1);
	MatrixXd value2(1,1);
	MatrixXd value3(1,1);
	MatrixXd value4(1,1);
	MatrixXd point1(3,1);
	MatrixXd point2(3,1);
	MatrixXd point3(3,1);
	MatrixXd point4(3,1);
	MatrixXd v_point1(3,1);
	MatrixXd v_point2(3,1);
	MatrixXd v_point3(3,1);
	MatrixXd v_point4(3,1);
	MatrixXd v_point1_norm(3,1);
	MatrixXd v_point2_norm(3,1);
	MatrixXd v_point3_norm(3,1);
	MatrixXd v_point4_norm(3,1);

	point1 <<h1*x1g + h2, h1*x1g+h2, 0;
	point2 <<h1*x1g+h2, h1*x2g+h2, 0;
	point3 <<h1*x2g+h2, h1*x1g+h2, 0;
	point4 <<h1*x2g+h2, h1*x2g+h2, 0;

	v_point1 = Cbi*v_i_3d + cross_op(w_b)*point1;
	v_point2 = Cbi*v_i_3d + cross_op(w_b)*point1;
	v_point3 = Cbi*v_i_3d + cross_op(w_b)*point1;
	v_point4 = Cbi*v_i_3d + cross_op(w_b)*point1;

	v_point1_norm = v_point1/v_point1.norm();
	v_point2_norm = v_point2/v_point2.norm();
	v_point3_norm = v_point3/v_point3.norm();
	v_point4_norm = v_point4/v_point4.norm();

	value1 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point1)*v_point1_norm)) ;
	value2 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point2)*v_point2_norm)) ;
	value3 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point3)*v_point3_norm)) ;
	value4 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point4)*v_point4_norm)) ;

	MatrixXd integral(1,1);

	integral = h1*h3 *( w1g*w1g*value1 + w1g*w2g*value2 + w2g*w1g*value3 + w2g*w2g* value4);


	if (v_i.norm()  < 0.001)
	{
		f_f = MatrixXd::Zero(3,1);
	}
	else{
		f_f = -Temp*nu*m*g*v_i/v_i.norm() ;
	}


	if (dtheta < 0.001){
		n_f << 0,0,0;
	}
	else
	{
		n_f <<0,0, integral(0);
	}

	MatrixXd f_friction(3,1);

	f_friction = f_f + n_f;
//

	MatrixXd dq_slider_next(3,1);
        MatrixXd q_slider_next(3,1);
        MatrixXd contact(2,1);
        double psi_next;
	MatrixXd w_b_next(1,1);
	MatrixXd Rotational_term(3,1);
	MatrixXd dr_pb_i(3,1);
	MatrixXd dpsi_vec(3,1);
        MatrixXd contact_vec(2,1);
	MatrixXd vc(3,1);
        MatrixXd Eye(2,2);
        Eye << 1,0,0,1;

	double cn   = u(0)*1+ 3.6155;
	double beta1= u(1)*0;
	double beta2= u(2)*0;
	double dpsi = u(3)*0;

        // theta=0;
	dq_slider_next << dq_slider*1 + h*M_inv*(f_friction + n*cn + d1*beta1 + d2*beta2 );
        // q_slider_next =q_slider + h*dq_slider_next;
        
	w_b_next = n3*dq_slider_next(2);
	Rotational_term = cross_op(w_b_next)*r_pb_i;
	dpsi_vec << 0, dpsi, 0;
        
	dr_pb_i = Cbi.transpose()*dpsi_vec;
	vc =  dq_slider_next + Rotational_term*1 + dr_pb_i*1;
// 
	// vp(0) = vc(0);
	// vp(1) = vc(1);
        vp(0) = vc(0);
	vp(1) = vc(1);
        
        // cout<< " vp " << vp << endl;
        // cout<< " dq_slider " << dq_slider << endl;
// 
        // psi_next = psi+h*dpsi*1;
        // contact_vec << xp_des, yp_des;
        // 
        // Cbi_T = Cbi.transpose();
        
        // cout<< Cbi_T.topLeftCorner(2,2)<<endl;
        
        // contact = Cbi_T.topLeftCorner(2,2)*contact_vec + q_slider_next.topLeftCorner(2,1);
        
        // contact = contact_vec + h*vp;

        // cout<< " x " << x << " y "<<y << " theta "<<theta <<endl;
        // cout<< " contact " << contact <<endl;
        
        
        // cout<< " dx " << dx << " dy "<<dy << " dtheta "<<dtheta <<endl;
        // cout<< " xp " << xp << " yp " <<yp;
        // cout<< " forces " << cn<<" "<<beta1<<" "<<beta2<<" "<<dpsi<<endl;
        // cout<< " vp_x " << vp(0) <<  " vp_y " << vp(1)<<endl;
        // cout<<" dq_slider_next "<<dq_slider_next<<endl;
        
	return vp;



}

//*************************************************************************
MatrixXd  cross_op(MatrixXd w)
{
	MatrixXd Omega(3,3);

	Omega << 0,-w(2),w(1),w(2),0,-w(0),-w(1),w(0),0;

	return Omega;
}


//********************************************************************
double gettime()
{

	char fmt[64];
	char buf[64];
	struct timeval tv;
	struct tm *tm;

	double test_sec;
	double test_usec;
	double test_usec2;
	double t_ini;

	gettimeofday(&tv, NULL);
	tm = localtime(&tv.tv_sec);
	test_sec = tv.tv_sec;
	test_usec = tv.tv_usec;
	test_usec2 = test_usec/(1000000);
	t_ini = test_sec+test_usec2;

	return t_ini;


}

//******************************
MatrixXd smooth(MatrixXd data, float filterVal, MatrixXd smoothedVal){


  if (filterVal > 1){      // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return smoothedVal;
}

//***************************************
void write_file(FILE* myFile, int num_rows, int num_cols, double *A)
{
    int i;
    int j;
    float value;


        const int RowOffset = (0 * num_cols);
        for (j=0;j<num_cols;j++)
        {;
        value = A[RowOffset + j] ;

        if (j==num_cols-1){
            fprintf(myFile, "%e \n",  value);}
        else
        {
            fprintf(myFile, "%e ",  value);


        }

    }

}



