/*
 */

using namespace std;
using Eigen::MatrixXd;
using Eigen::ArrayXf;

#define NUM_VARIABLES 385
#define NUM_INEQ_CONSTRAINTS 630
#define NUM_EQ_CONSTRAINTS 245
#define NUM_CONSTRAINTS 875

#ifndef PUSHING_H
#define PUSHING_H

extern const int num_ineq_constraints;
extern const int num_eq_constraints;
extern const int num_variables;
extern const int num_constraints;

// *****************************
struct thread_data{
    MatrixXd  *_q_pusher;
    MatrixXd  *_q_slider;
    MatrixXd  *_dq_slider;
    MatrixXd  *_Target;
    MatrixXd  *_u_control;
    MatrixXd  *_ap;
    double  *_tang_vel ;
} ;

class Push {

	double  Aeq[NUM_EQ_CONSTRAINTS][NUM_VARIABLES];
	double  beq[NUM_EQ_CONSTRAINTS];
	double  Ain[NUM_INEQ_CONSTRAINTS][NUM_VARIABLES];
	double  bin[NUM_INEQ_CONSTRAINTS];
	double  Atotal[NUM_CONSTRAINTS][NUM_VARIABLES];
	double  btotal[NUM_CONSTRAINTS];
	double variables[NUM_VARIABLES];

	double  c[NUM_CONSTRAINTS];
	double  Q[NUM_VARIABLES][NUM_VARIABLES];
	double  Abar[7][7];
	double  lb_stick[NUM_VARIABLES];
	double  lb_up[NUM_VARIABLES];
	double  lb[NUM_VARIABLES];
	double  ub_stick[NUM_VARIABLES];
	double  ub_up[NUM_VARIABLES];
	double  ub[NUM_VARIABLES];
	char    vtype_stick[NUM_VARIABLES];
	char    vtype_up[NUM_VARIABLES];
	char    vtype[NUM_VARIABLES];
	double solution[];
	bool    success_stick;
	bool    success_up;
	bool    success;
	double  objval_stick, sol_stick[NUM_VARIABLES];
	double  objval_up, sol_up[NUM_VARIABLES];
	double  objval, sol[NUM_VARIABLES];
	char    sense_stick[NUM_CONSTRAINTS];
	char    sense_up[NUM_CONSTRAINTS];
	char    sense[NUM_CONSTRAINTS];
	char    *_Q_cost;
	char    *_Ain;
	char    *_bin;
	char    *_Aeq;
	char    *_beq;
	char    *_Abar_str;
	int _index;

        GRBEnv env;
	GRBModel model;
	GRBVar* vars;
	GRBLinExpr lhs;
	GRBConstr constr[NUM_CONSTRAINTS] ;

	double nu;
	double nu_pusher;
	double m ;
	double J ;
	double a ;
	double b ;
	double A ;
	double g ;
	double h ; // Think about this!
	double psi;
	double theta;
	double dtheta;

	MatrixXd q_slider;
	MatrixXd dq_slider;
	MatrixXd q_pusher;
	MatrixXd dq_pusher;
	MatrixXd vp;
	MatrixXd A_bar_temp;
	MatrixXd delta_x_temp;
	MatrixXd b_temp;
	MatrixXd x_ref;
	MatrixXd x_sensor;
	MatrixXd r_cb_b;
	MatrixXd Cbi;
	MatrixXd Cbi_T;
	MatrixXd gi;
	MatrixXd gb;
	MatrixXd theta_rel;
	MatrixXd theta_g;
	MatrixXd Cci;
	MatrixXd rc;
	MatrixXd vc;
	MatrixXd xc, yc, dxc, dyc;
	MatrixXd u;

//	Initialize text file variable
	FILE *myFile = NULL;

private:
	void read_file(FILE*, int, int, double*);

public:
	Push (char*, char*, char*, char*, char*, char*);
	void set_equation_sense();
	void set_variable_type();
	void read_array();
	void build_model();
	void stack_arrays();
	double optimize();
	void update_model(MatrixXd , MatrixXd, MatrixXd, MatrixXd);
	MatrixXd inverse_dynamics();
	MatrixXd delta_u;

};

// MatrixXd cross_op(MatrixXd w);
// MatrixXd inverse_dynamics2(MatrixXd q_pusher, MatrixXd q_slider, MatrixXd dq_slider, MatrixXd u, double tang_vel, double time);
// double gettime();
// double smooth(double data, float filterVal, double smoothedVal);
// void write_file(FILE* myFile, int num_rows, int num_cols, double *A);
#endif



