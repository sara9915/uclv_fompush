/*
 */

using namespace std;
using Eigen::MatrixXd;
using Eigen::ArrayXf;

#define NUM_VARIABLES 385
#define NUM_INEQ_CONSTRAINTS 630
#define NUM_EQ_CONSTRAINTS 245
#define NUM_CONSTRAINTS 875

#ifndef HELPER_H
#define HELPER_H

MatrixXd cross_op(MatrixXd w);
MatrixXd inverse_dynamics(MatrixXd q_pusher, MatrixXd q_slider, MatrixXd dq_slider, MatrixXd u, double tang_vel, double time);
double gettime();
double smooth(double data, float filterVal, double smoothedVal);
void write_file(FILE* myFile, int num_rows, int num_cols, double *A);




#endif


