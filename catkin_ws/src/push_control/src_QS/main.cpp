//~ //System
#include  "HeaderFiles.h"

//Define shortcuts
using namespace abb::egm;
using namespace tf;
using namespace std;
using Eigen::MatrixXd;
//***********************Define Global Variables*************************************
//Thread
pthread_mutex_t nonBlockMutex;
//Structures
struct thread_data thread_data_array[1];
//Doubles
double TimeGlobal=0;
double Flag=0;
//Vector
std::vector<geometry_msgs::WrenchStamped> ft_wrenches;
//JSON Variables
Json::StyledWriter styledWriter;
Json::Value JsonOutput;
Json::Value timeJSON;
Json::Value q_sliderJSON;
Json::Value q_pusher_sensedJSON;
Json::Value q_pusher_commandedJSON;
Json::Value u_controlJSON;
Json::Value u_controlMPCJSON;
Json::Value delta_xMPCJSON;
Json::Value vipiJSON;
//*********************** Main Program *************************************
int main(int argc,  char *argv[]){
    //~Ros parameters---------------------------------------------------------------------------------------
    ros::init(argc, argv, "push_control");
    ros::NodeHandle n;
    tf::TransformListener listener;
    //~ ros::Subscriber sub = n.subscribe("/netft_data", 1, chatterCallback);
    //********Define Local Variables-------------------------------------------------------------------------
    //Mutex
    pthread_mutex_init(&nonBlockMutex, NULL);
    pthread_t rriThread;
    pthread_attr_t attrR;
    pthread_attr_init(&attrR);
    pthread_attr_setdetachstate(&attrR, PTHREAD_CREATE_JOINABLE);
    //Matrices
    MatrixXd q_pusher(2,1);
    MatrixXd _q_pusher_sensor(2,1);
    MatrixXd q_slider(3,1);
    MatrixXd u_control(2,1);
    MatrixXd u_controlMPC(2,35);
    MatrixXd delta_xMPC(4,35);
    MatrixXd _q_pusher(2,1);
    MatrixXd _q_slider(3,1);
    MatrixXd _u_control(2,1);
	MatrixXd _u_controlMPC(2,35);
	MatrixXd _delta_xMPC(4,35);
    MatrixXd vipi(2,1);
    MatrixXd vbpi(2,1);
    MatrixXd Cbi(2,2);
    //Integers
    int tmp = 0;
    int lv1 = 0;
    //Doubles
    double  z_tcp, x_tcp, y_tcp;
    double  _z_tcp, _x_tcp, _y_tcp;
    double  time, t_ini;
    double theta;
    double h=1.0f/1000;
    //Booleans
    bool has_robot = false;
    bool has_vicon_pos = false;
    bool has_vicon_vel = false;
    //Variable to pass to thread
    thread_data_array[0]._q_pusher = &q_pusher;
    thread_data_array[0]._q_slider = &q_slider;
    thread_data_array[0]._u_control = &u_control;
    thread_data_array[0]._u_controlMPC = &u_controlMPC;
    thread_data_array[0]._delta_xMPC = &delta_xMPC;
    // Create socket and wait for robot's connection
    UDPSocket* EGMsock;
    const int portNumber = 6510;
    string sourceAddress;             // Address of datagram source
    unsigned short sourcePort;        // Port of datagram source
    EGMsock = new UDPSocket(portNumber);
    EgmSensor *pSensorMessage = new EgmSensor();
    EgmRobot *pRobotMessage = new EgmRobot();
    string messageBuffer;
    
   
    //-------------------------------------------------------------------------------------------------------------------------------
    //First Loop (Check Vicon, Robot connection, ros) || (tmp < 2500) && ros::ok()
    while(!has_robot || !has_vicon_pos || (tmp < 2500) && ros::ok())
    {
        tmp++;
        //Read robot position
        cout << " In first loop" << endl;
        cout << " tmp " << tmp << endl;
        cout << " has_vicon_pos "<< has_vicon_pos << endl;
        cout << " has_robot "    << has_robot << endl;
        cout << " ros::ok() " << ros::ok() << endl;
         
        if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, x_tcp, y_tcp, z_tcp)){
            has_robot = true;
            CreateSensorMessage(pSensorMessage,0.15,0);
            pSensorMessage->SerializeToString(&messageBuffer);
            EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        }
        if(getViconPose(q_slider, listener)){
            has_vicon_pos = true;
            ros::spinOnce();
            lv1+=1;
            usleep(4e3);}
        // if (tmp>10000){break;}
    }
    // return 0;
    //Initialize q_pusher
    q_pusher(0) = x_tcp;
    q_pusher(1) = y_tcp;
    
    //Print q_pusher and q_slider
    // cout<<"q_pusher"<<endl;
    // cout<<q_pusher<<endl;
    // cout<<"q_slider"<<endl;
    // cout<<q_slider<<endl;
     // return 0;

    //Create Thread------------------------------------------------------------------------------------------------------
    pthread_create(&rriThread, &attrR, rriMain, (void*) &thread_data_array[0]);
    //Second Loop (Give time to thread to initialize)-------------------------------------------------------------------
    for(int i=0;i<1000;i++){
        cout << " In Second loop " << i << endl;
        if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, x_tcp, y_tcp, z_tcp)){
            has_robot = true;
            CreateSensorMessageEmpty(pSensorMessage);
            pSensorMessage->SerializeToString(&messageBuffer);
            EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        }
        ros::spinOnce();
        usleep(4e3);
    }
    //Intermediate communication step (NOt required, helped with robot communication)----------------------------------------------
    if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, _x_tcp, _y_tcp, _z_tcp)){
        _q_pusher_sensor<<_x_tcp,_y_tcp;
        }
        // cout<< "u_control"<<endl;
        // cout<< u_control<<endl; 
        // return 0;
    //*************************************************************************************************************************
    //************** Main Control Loop ****************************************************************************************
    //*************************************************************************************************************************
    ros::Rate r(1000);
    for (int i =0;i<150000 && ros::ok();i++)
    {
        //Get time---------------------------------------------------------------------------------------------------------------------
        if (i==0){t_ini = gettime();}
        time = gettime()- t_ini;
        //Read data and store----------------------------------------------------------------------------------------------------------
        if(getRobotPose(EGMsock, sourceAddress, sourcePort, pRobotMessage, _x_tcp, _y_tcp, _z_tcp)){
           _q_pusher_sensor<<_x_tcp,_y_tcp;
        }  
        pthread_mutex_lock(&nonBlockMutex);
        getViconPose(q_slider, listener);
        // cout<< "q_slider"<<endl;
        // cout<< q_slider<<endl;
        printf ("q_slider %f %f %f \n",q_slider(0),q_slider(1),q_slider(2));
        q_pusher(0) = x_tcp;// + tcp_width*cos(theta*1);
        q_pusher(1) = y_tcp;// + tcp_width*sin(theta*1);
        // q_pusher=_q_pusher_sensor;
        //Assign local variables
        _q_slider = q_slider;
        _q_pusher = q_pusher;
        _u_control = u_control;
        _u_controlMPC = u_controlMPC;
        _delta_xMPC = delta_xMPC;
        TimeGlobal = time;
        // cout<< "u_control"<<endl;
        // cout<< u_control<<endl;
        pthread_mutex_unlock(&nonBlockMutex);
        //Position Control Parameters --------------------------------------------------------------------------------------------------
        if (time<=1)
          {x_tcp = x_tcp;
          }
        else{    
            if (x_tcp>0.55 or Flag==3){
                vipi(0) = 0;
                vipi(1) = 0;}
            else{
                //Convert u_control from body to intertial reference frame
                theta = _q_slider(2);
                Cbi<< cos(theta), sin(theta), -sin(theta), cos(theta);
                vbpi(0) = _u_control(0)*1 + 0.05*1;
                vbpi(1) = _u_control(1)*1;
                vipi = Cbi.transpose()*vbpi;
            }
            x_tcp = x_tcp + h*vbpi(0);
            y_tcp = y_tcp + h*vbpi(1);
            
            // Update JSON Arrays
            timeJSON.append(time);
            q_pusher_sensedJSON[0].append(_x_tcp);
            q_pusher_sensedJSON[1].append(_y_tcp);
            q_pusher_commandedJSON[0].append(x_tcp);
            q_pusher_commandedJSON[1].append(y_tcp);
            for (int j =0;j<3;j++)
			{
				q_sliderJSON[j].append(_q_slider(j));
			}
			
			for (int j =0;j<2;j++)
			{
				u_controlJSON[j].append(_u_control(j));
				vipiJSON[j].append(vipi(j));
			}
        }
        // cout<< x_tcp<<endl;
        CreateSensorMessage(pSensorMessage, x_tcp, y_tcp);
        pSensorMessage->SerializeToString(&messageBuffer);
        EGMsock->sendTo(messageBuffer.c_str(), messageBuffer.length(), sourceAddress, sourcePort);
        
        //Sleep for 1000Hz loop
        // usleep(1000);
        r.sleep();
    }
    
    // Save JSON Output file
    JsonOutput["timeJSON"] = timeJSON;
    JsonOutput["q_sliderJSON"] = q_sliderJSON;
    JsonOutput["q_pusher_sensedJSON"] = q_pusher_sensedJSON;
    JsonOutput["q_pusher_commandedJSON"] = q_pusher_commandedJSON;
    JsonOutput["u_controlJSON"] = u_controlJSON;
    JsonOutput["vipiJSON"] = vipiJSON;
    
    ofstream myOutput;
    myOutput.open ("/home/mcube/cpush/catkin_ws/src/push_control/data/Test.json");
    myOutput << styledWriter.write(JsonOutput);
    myOutput.close();
    
    
    cout<< "End of Program"<<endl;
    
    return 0;
}
