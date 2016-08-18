/*
 */

using namespace std;
using Eigen::MatrixXd;
using Eigen::ArrayXf;

#define NUM_VARIABLES 385
#define NUM_INEQ_CONSTRAINTS 630
 #include "json/json.h"
 
#define NUM_EQ_CONSTRAINTS 245
#define NUM_CONSTRAINTS 875

#ifndef HELPER_H
#define HELPER_H

extern Json::Value cnOut;
extern Json::Value beta1Out;
extern Json::Value beta2Out;
extern Json::Value dpsiOut;
extern Json::Value psiOut;
extern Json::Value aoxOut;
extern Json::Value aoyOut;
extern Json::Value aozOut;
extern Json::Value aipixOut;
extern Json::Value aipiyOut;
extern Json::Value aipizOut;
extern Json::Value abpbxOut;
extern Json::Value abpbyOut;
extern Json::Value rbpbxOut;
extern Json::Value rbpbyOut;
extern Json::Value vbpbxOut;
extern Json::Value vbpbyOut;
extern Json::Value fFrictionxOut;
extern Json::Value fFrictionyOut;
extern Json::Value fFrictionzOut;
//
extern Json::Value JsonOutput;
extern Json::Value timeOut;
extern Json::Value qSliderxOut;
extern Json::Value qSlideryOut;
extern Json::Value qSliderzOut;
extern Json::Value dqSliderxOut;
extern Json::Value dqSlideryOut;
extern Json::Value dqSliderzOut;
extern Json::Value _x_tcpOut;
extern Json::Value _y_tcpOut;
extern Json::Value x_tcpOut;
extern Json::Value y_tcpOut;
extern Json::Value vpxOut;
extern Json::Value vpyOut;
extern Json::Value apxOut;
extern Json::Value apyOut;
extern Json::Value fxOut;
extern Json::Value fyOut;
extern Json::Value fzOut;
extern Json::Value fxBiasOut;
extern Json::Value fyBiasOut;
extern Json::Value fzBiasOut;
extern Json::Value fxIniOut;
extern Json::Value fyIniOut;
extern Json::Value fzIniOut;
//
extern Json::StyledWriter styledWriter;

MatrixXd cross_op(MatrixXd w);
struct OutputData inverse_dynamics(MatrixXd q_pusher, MatrixXd q_slider, MatrixXd dq_slider, MatrixXd u, double tang_vel, double time);
double gettime();
double smooth(double data, float filterVal, double smoothedVal);
void write_file(FILE* myFile, int num_rows, int num_cols, double *A);
void updateJSON_data(double time, MatrixXd q_slider, MatrixXd dq_slider, double _x_tcp, double _y_tcp, double x_tcp, double y_tcp, MatrixXd vp, MatrixXd ap, double fx, double fy, double fz, struct OutputData Output);

void outputJSON_file();


#endif


