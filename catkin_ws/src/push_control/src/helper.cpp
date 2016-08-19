#include <iostream>
#include <string>
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <fstream>
#include <memory>
#include <cstdlib>
#include "pushing.h"
#include "helper.h"
#include "ABBRobot.h"
#include "OptProgram.h"
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

//********************************************************************************
OutputData inverse_dynamics(MatrixXd q_pusher, MatrixXd q_slider, MatrixXd dq_slider, MatrixXd u, double tang_vel, double time)
{
        //Declare constant parameters
	const double nu = 0.35;
	const double nu_pusher = 0.3;
	const double m = 1.0530;
	const double J = 0.00142155;
	const double a = 0.09;
	const double b = 0.09;
	const double A = a*b;
	const double g = 9.81;
	const double h = 1.f/1000.0; // Think about this!

        // Declare function variables
	MatrixXd M_inv(3,3);
	MatrixXd Cbi(3,3);
	MatrixXd Cbi2d(2,2);
        MatrixXd Cbi_T(3,3);
	MatrixXd w_b(3,1);
	MatrixXd w_i(3,1);
	MatrixXd rbpb(3,1);
	MatrixXd ripb(3,1);
	MatrixXd ripb2d(2,1);
	MatrixXd ribi(2,1);
	MatrixXd ripb_des(2,1);
	MatrixXd rici(2,1);
	MatrixXd n(3,1);
	MatrixXd d1(3,1);
	MatrixXd d2(3,1);
	MatrixXd n_cart(3,1);
	MatrixXd d1_cart(3,1);
	MatrixXd d2_cart(3,1);
	MatrixXd vp(2,1);
        MatrixXd ap(8,1);
        MatrixXd ap_3x1(3,1);
        MatrixXd ap_2x1(2,1);
        MatrixXd aibi(3,1);
        MatrixXd vibi(3,1);
        MatrixXd aipi(3,1);
        MatrixXd vipi(3,1);
        MatrixXd abpb(3,1);
        MatrixXd vbpb(3,1);
        MatrixXd wbbi(3,1);
        MatrixXd dwbbi(3,1);
        MatrixXd n3(3,1);
        
	double x = q_slider(0);
	double y = q_slider(1);
	double theta = q_slider(2);
	double dx = dq_slider(0);
	double dy = dq_slider(1);
	double dtheta = dq_slider(2);
	double xp = q_pusher(0);
	double yp = q_pusher(1);

        //Define Kinematic relations
        M_inv << 1/m,0,0,0,1/m,0,0,0,1/J;
        n3<<0,0,1;
	w_b << 0,0,dtheta;
	wbbi << 0,0,dtheta;
	ribi << x,y;
	ripb << xp-x,yp-y,0;

        Cbi2d << cos(theta), sin(theta), -sin(theta), cos(theta);
        Cbi << cos(theta), sin(theta), 0, -sin(theta), cos(theta), 0, 0, 0, 1;
        Cbi_T = Cbi.transpose();
        
	w_i = Cbi.transpose()*w_b;
	rbpb = Cbi*ripb;
        rbpb(0) = -0.045;
        
        //Define variables
	double psi =rbpb(1);
	MatrixXd f_f(3,1);
	MatrixXd Temp(3,2);
	MatrixXd v_i(2,1);
	MatrixXd v_i_3d(3,1);
	MatrixXd n_f(3,1);
        MatrixXd qp_temp(3,1);
        
        //Find ideal location of pusher
        rbpb2d(0) = rbpb(0);
        rbpb2d(1) = rbpb(1);
        ripb_des = Cbi2d.transpose()*rbpb2d;
        rici = ribi - ripb_des;
        
        //Define useful kinematic relationships
        n <<   cos(theta), sin(theta), -psi;
        d1 << -sin(theta), cos(theta), -0.045;
        d2 << sin(theta), -cos(theta),0.045;

        // Kinematic relations
	v_i << dx,dy;
	v_i_3d << dx,dy,0;
	Temp << 1,0,0,1,0,0;
        
        ///////////////////////////// Integration ///////////////////////////////////////////////////
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

	double x1g = sqrt(1.0f/3);
	double x2g = -sqrt(1.0f/3);

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

	point1 <<h1*x1g+h2, h1*x1g+h2, 0;
	point2 <<h1*x1g+h2, h1*x2g+h2, 0;
	point3 <<h1*x2g+h2, h1*x1g+h2, 0;
	point4 <<h1*x2g+h2, h1*x2g+h2, 0;

	v_point1 = Cbi*v_i_3d + cross_op(w_b)*point1;
	v_point2 = Cbi*v_i_3d + cross_op(w_b)*point2;
	v_point3 = Cbi*v_i_3d + cross_op(w_b)*point3;
	v_point4 = Cbi*v_i_3d + cross_op(w_b)*point4;

	v_point1_norm = v_point1/v_point1.norm();
	v_point2_norm = v_point2/v_point2.norm();
	v_point3_norm = v_point3/v_point3.norm();
	v_point4_norm = v_point4/v_point4.norm();

	value1 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point1)*v_point1_norm)) ;
	value2 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point2)*v_point2_norm)) ;
	value3 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point3)*v_point3_norm)) ;
	value4 = n3.transpose()*(-(nu*m*g)/A* (cross_op(point4)*v_point4_norm)) ;

	MatrixXd integral(1,1);
        MatrixXd velocity_initial(2,1);

	integral = h1*h3 *( w1g*w1g*value1 + w1g*w2g*value2 + w2g*w1g*value3 + w2g*w2g* value4);

         /////////////////////////////// Compute Frictional Forces ///////////////////////////////////////////////////
	if (v_i.norm()  < 0.001)
	{
		// f_f = MatrixXd::Zero(3,1);
                velocity_initial<<n(0),n(1);
                f_f = -Temp*nu*m*g*velocity_initial ;
	}
	else{
		f_f = -Temp*nu*m*g*v_i/v_i.norm() ;
	}

        
	if (abs(dtheta) < 0.001){
		n_f << 0,0,0;
	}
	else
	{
		n_f <<0,0, integral(0)*1;
	}

	MatrixXd f_friction(3,1);
	f_friction = f_f + n_f;
        
        //Declare variables
	MatrixXd dq_slider_next(3,1);
        MatrixXd q_slider_next(3,1);
        double psi_des = 0;
        MatrixXd contact(2,1);
        double psi_next;
	MatrixXd w_b_next(3,1);
        MatrixXd dw(3,1);
	MatrixXd Rotational_term(3,1);
	MatrixXd dr_pb_i(3,1);
	MatrixXd rpb_b_des(3,1);
	MatrixXd rpb_i_des(3,1);
	MatrixXd dpsi_vec(3,1);
        MatrixXd contact_vec(2,1);
	MatrixXd vc(3,1);
        MatrixXd Eye(2,2);
        MatrixXd ddq(3,1);
        MatrixXd ao(3,1);
        MatrixXd an(3,1);
        MatrixXd at(3,1);
        MatrixXd vp_temp(3,1);
        Eye << 1,0,0,1;

	double cn   = u(0)*1+3.6155;////3.6155;
	double beta1= u(1)*1;
	double beta2= u(2)*1;
	double dpsi = u(3)*1;
       
        // Compute object acceleration
        ao = M_inv*(f_friction + n*cn + d1*beta1 + d2*beta2 );
        aibi << ao(0), ao(1), 0;
        abpb << 0, (dpsi-tang_vel)/h,0;
        dwbbi << 0,0,ao(2);
        vbpb << 0,dpsi,0;

        aipi = aibi+ Cbi.transpose()*abpb + Cbi.transpose()*cross_op(dwbbi)*rbpb*1 + 2*Cbi.transpose()*cross_op(wbbi)*vbpb + Cbi.transpose()*cross_op(wbbi)*cross_op(wbbi)*rbpb;
  
        
          
        struct OutputData Output_IK;

        Output_IK.aipi = aipi;
        Output_IK.psi = psi;
        Output_IK.cn = cn;
        Output_IK.beta1 = beta1;
        Output_IK.beta2 = beta2;
        Output_IK.dpsi = dpsi;
        Output_IK.ao  = ao;
        Output_IK.abpb  = abpb;
        Output_IK.rbpb  = rbpb;
        Output_IK.vbpb  = vbpb;
        Output_IK.fFriction  = f_friction;
        Output_IK.rici  = rici;

	return Output_IK;
}

void constraintRobotPusher(double &x_tcp, double &y_tcp, MatrixXd q_slider, struct OutputData Output){
        //Declare variables
        MatrixXd Cbi(2,2);
        MatrixXd ripi(2,1);
        MatrixXd rbpi(2,1);
        MatrixXd ripi_clamp(2,1);
        MatrixXd rbpi_clamp(2,1);
        MatrixXd rbci(2,1);
        
        cout<< "Initial"<<x_tcp<<y_tcp<<endl;
     
        double delta_x = 0.01;
        double delta_y = 0.01;
        double theta;
        //Assign Values
        theta = q_slider(2);
        ripi<< x_tcp, y_tcp;
        Cbi << cos(theta), sin(theta), -sin(theta), cos(theta);
        //Transform to Fb
        rbpi = Cbi*ripi;
        rbci = Cbi*Output.rici;
        //Clamp pusher x position in Fb
        if (rbpi(0) > rbci(0)+delta_x){
                rbpi_clamp(0) = rbci(0)+delta_x;}
        else if (rbpi(0)< rbci(1)){
        rbpi_clamp(0) = rbci(0);}
        else{
        rbpi_clamp(0) = rbpi(0);}
        //Clamp pusher y position in Fb
        if (rbpi(1) > rbci(1)+delta_y){
                rbpi_clamp(1) = rbci(1)+delta_y;}
        else if (rbpi(1)< rbci(1) - delta_y){
        rbpi_clamp(1) = rbci(1)-delta_y;}
        else{
        rbpi_clamp(1) = rbpi(1);}
        //Resolve in world frame
        ripi_clamp = Cbi.transpose()*rbpi_clamp;
        //Overwrite commanded position of pusher
        x_tcp = ripi_clamp(0);
        y_tcp = ripi_clamp(1);
        
        cout<< "Final"<<x_tcp<<y_tcp<<rbci(0)<<rbci(1)<<endl;

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
double smooth(double data, float filterVal, double smoothedVal){


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
//******************
void updateJSON_data(double time, MatrixXd q_slider, MatrixXd dq_slider, double _x_tcp, double _y_tcp, double x_tcp, double y_tcp, MatrixXd vp, MatrixXd ap, double fx, double fy, double fz, struct OutputData Output){
        
        timeOut.append(time);
        qSliderxOut.append(q_slider(0));
        qSlideryOut.append(q_slider(1));
        qSliderzOut.append(q_slider(2));
        dqSliderxOut.append(dq_slider(0));
        dqSlideryOut.append(dq_slider(1));
        dqSliderzOut.append(dq_slider(2));
        _x_tcpOut.append(_x_tcp);
        _y_tcpOut.append(_y_tcp);
        x_tcpOut.append(x_tcp);
        y_tcpOut.append(y_tcp);
        vpxOut.append(vp(0));
        vpyOut.append(vp(1));
        apxOut.append(ap(0));
        apyOut.append(ap(1));
        fxOut.append(fx);
        fyOut.append(fy);
        fzOut.append(fz);
        // fxBiasOut.append(contact_wrench_bias.wrench.force.x);
        // fyBiasOut.append(contact_wrench_bias.wrench.force.y);
        // fzBiasOut.append(contact_wrench_bias.wrench.force.z);
        // fxIniOut.append(contact_wrench_ini.wrench.force.x);
        // fyIniOut.append(contact_wrench_ini.wrench.force.y);
        // fzIniOut.append(contact_wrench_ini.wrench.force.z);
        
        cnOut.append(Output.cn);
        beta1Out.append(Output.beta1);
        beta2Out.append(Output.beta2);
        dpsiOut.append(Output.dpsi);
        psiOut.append(Output.psi);
        aoxOut.append(Output.ao(0));
        aoyOut.append(Output.ao(1));
        aozOut.append(Output.ao(2));
        aipixOut.append(Output.aipi(0));
        aipiyOut.append(Output.aipi(1));
        aipizOut.append(Output.aipi(2));
        abpbxOut.append(Output.abpb(0));
        abpbyOut.append(Output.abpb(1));
        rbpbxOut.append(Output.rbpb(0));
        rbpbyOut.append(Output.rbpb(1));
        vbpbxOut.append(Output.vbpb(0));
        vbpbyOut.append(Output.vbpb(1));
        fFrictionxOut.append(Output.fFriction(0));
        fFrictionyOut.append(Output.fFriction(1));
        fFrictionzOut.append(Output.fFriction(2));
        
}

void outputJSON_file(){
        //Format output json file
        JsonOutput["time"] = timeOut;
        JsonOutput["q_sliderX"] = qSliderxOut;
        JsonOutput["q_sliderY"] = qSlideryOut;
        JsonOutput["q_sliderZ"] = qSliderzOut;
        JsonOutput["dq_sliderX"] = dqSliderxOut;
        JsonOutput["dq_sliderY"] = dqSlideryOut;
        JsonOutput["dq_sliderZ"] = dqSliderzOut;
        JsonOutput["x_tcpSensor"] = _x_tcpOut;
        JsonOutput["y_tcpSensor"] = _y_tcpOut;
        JsonOutput["x_tcp"] = x_tcpOut;
        JsonOutput["y_tcp"] = y_tcpOut;
        JsonOutput["vpX"] = vpxOut;
        JsonOutput["vpY"] = vpyOut;
        JsonOutput["apX"] = apxOut;
        JsonOutput["apY"] = apyOut;
        JsonOutput["fx"] = fxOut;
        JsonOutput["fy"] = fyOut;
        JsonOutput["fz"] = fzOut;
        // JsonOutput["fxBias"] = fxBiasOut;
        // JsonOutput["fyBias"] = fyBiasOut;
        // JsonOutput["fzBias"] = fzBiasOut;
        // JsonOutput["fxIni"] = fxIniOut;
        // JsonOutput["fyIni"] = fyIniOut;
        // JsonOutput["fzIni"] = fzIniOut;
        JsonOutput["cn"] = cnOut;
        JsonOutput["beta1"] = beta1Out;
        JsonOutput["beta2"] = beta2Out;
        JsonOutput["dpsi"] = dpsiOut;
        JsonOutput["psi"] = psiOut;
        JsonOutput["aoX"] = aoxOut;
        JsonOutput["aoY"] = aoyOut;
        JsonOutput["aoZ"] = aozOut;
        JsonOutput["aipiX"] = aipixOut;
        JsonOutput["aipiY"] = aipiyOut;
        JsonOutput["aipiZ"] = aipizOut;
        JsonOutput["abpbX"] = abpbxOut;
        JsonOutput["abpbY"] = abpbyOut;
        JsonOutput["rbpbX"] = rbpbxOut;
        JsonOutput["rbpbY"] = rbpbyOut;
        JsonOutput["vbpbX"] = vbpbxOut;
        JsonOutput["vbpbY"] = vbpbyOut;
        JsonOutput["fFrictionX"] = fFrictionxOut;
        JsonOutput["fFrictionY"] = fFrictionyOut;
        JsonOutput["fFrictionZ"] = fFrictionzOut;       

        ofstream myOutput;
        myOutput.open ("OutputControllerWihRadius.json");
        myOutput << styledWriter.write(JsonOutput);
        myOutput.close();

}



