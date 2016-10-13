#include  "HeaderFiles.h"

using namespace abb::egm;
using namespace tf;
using namespace std;
using Eigen::MatrixXd;

//********************************************************************
// Optimization Thread
//********************************************************************
void *rriMain(void *thread_arg)
{
    struct thread_data *my_data;
    my_data = (struct thread_data *) thread_arg;

    //~ //Define variables from argument pointers
    pthread_mutex_lock(&nonBlockMutex);

    MatrixXd *pq_slider = my_data->_q_slider;
    MatrixXd *pq_pusher = my_data->_q_pusher;
    MatrixXd *pu_control = my_data->_u_control;
    MatrixXd &q_slider = *pq_slider;
    MatrixXd &q_pusher = *pq_pusher;
    MatrixXd &u_control = *pu_control;

    pthread_mutex_unlock(&nonBlockMutex);
    
    //~ //Define local variables
    double fval1;
    double fval2;
    double fval3;
    double t_ini;
    double time = 0;
    double counter = 0;
    int minIndex, maxCol;
    float min;
    //Define local matrix variables
    MatrixXd fval(3,1);
    MatrixXd _q_slider_(3,1);
    MatrixXd _q_pusher_(2,1);
    //Define object for 3 family of modes
    Push * pStick;
    Push * pUp;
    Push * pDown;
    pStick = new Push (1);
    pUp    = new Push (2);
    pDown  = new Push (3);
    Push &Stick= *pStick;
    Push &Up   = *pUp;
    Push &Down = *pDown;
    //**********************************************************************
    //************************ Begin Loop **********************************
    //**********************************************************************
    while(time<50000 && ros::ok())
        {
        if (time==0){t_ini = gettime();}
        time = gettime()- t_ini;     
        //Read state of robot and object from shared variables
        pthread_mutex_lock(&nonBlockMutex);       
        _q_slider_ = q_slider;
        _q_pusher_= q_pusher;
        pthread_mutex_unlock(&nonBlockMutex);
        //Update Model
        Stick.UpdateICModel(TimeGlobal,_q_slider_,_q_pusher_);
        Down.UpdateICModel(TimeGlobal,_q_slider_,_q_pusher_);
        Up.UpdateICModel(TimeGlobal,_q_slider_,_q_pusher_);
        //Optimize Models
        fval1 = Stick.OptimizeModel();
        fval2 = Up.OptimizeModel();
        fval3 = Down.OptimizeModel();
        //Find best control input
        fval << fval1, fval2, fval3;
        min = fval.minCoeff(&minIndex, &maxCol); 
        //Assign new control input to shared variables
        pthread_mutex_lock(&nonBlockMutex);
        // cout<<"Stick"<<endl;
        // cout<<Stick.delta_u<<endl;
        // cout<<fval1<<endl;
        // cout<<"Up"<<endl;
        // cout<<Up.delta_u<<endl;
        // cout<<fval2<<endl;
        // cout<<"Down"<<endl;
        // cout<<Down.delta_u<<endl;
        cout<<fval3<<endl;
        if (minIndex==0){ u_control = Stick.delta_u; 
            }
        else if (minIndex==1){  u_control = Up.delta_u;
            }
        else{   u_control = Down.delta_u;
            }
        pthread_mutex_unlock(&nonBlockMutex);
        //Remove Contraints
        Stick.RemoveConstraints();
        Up.RemoveConstraints();
        Down.RemoveConstraints();
        counter++;
        }
    //*********** End Loop **************************************************
    pthread_exit((void*) 0);
}

