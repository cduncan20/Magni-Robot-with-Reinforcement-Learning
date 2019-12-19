// Authors: Abhilesh Borode, Rachel Breshears, Casey Duncan, Emerson Ham, & Mohammah Aun Siddique
// Date: 5/1/2019
// This code trains the RL Q-Table for the Magni Robot by giving it random actions, calculating
// its state based on that action, and giving a reward value based on the robots state given
// the random action. After a sufficient number of iterations assigning random actions, the RL
// algorithm builds a Q-Table that allows the robot to learn how to travel in a straight line.

# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
# include <nav_msgs/Odometry.h>
# include <math.h> 
# include <fstream>

# define PI  3.14159265

using namespace std;
double g_x;
double g_y;
double g_z;
double combinedAngle;

double publishAction(int x)
{	double rad;
	double action[]= {-9,-3,-2,-1,0,1,2,3,9};
	rad = 6*action[x]* (PI /180);
	return rad;
}

// This has been modified for x being fwd and y perpendicular to the bot 
double state (double x_w,  double y_w)
{
	// x_w and y_w are the position of the robot in the world frame 
	double x_pos = x_w;
	double x_end = 4.0;
	double ref_x = 0.0;
	if (x_pos <= x_end -0.25){
		ref_x = x_pos+0.25;
	}
	else{
		ref_x = x_end;
	}

	double alpha = atan2(y_w,ref_x - x_pos)*180/PI;

	return alpha;
}



void poseMsgReceived( const  nav_msgs::Odometry  & msg){
	g_x = msg.pose.pose.position.x;
	g_y = msg.pose.pose.position.y;
	g_z = msg.pose.pose.position.z;

	// alpha is the combined angle
	//double alpha= 0;
	double alpha = state(g_x, g_y);
	combinedAngle = alpha;

	//ROS_INFO_STREAM("combined angle:    "<<alpha);
	//ROS_INFO_STREAM("linear x"<<  g_x);
	//ROS_INFO_STREAM("linear y"<<  g_y);
}

int getState(double combinedAngle)
{	
	int stateindex;
	if( combinedAngle < -8)
 	stateindex = 9;
	else if( combinedAngle < -4)
 	stateindex = 8;
	else if( combinedAngle < -2)
 	stateindex = 7;
	else if( combinedAngle < -1)
 	stateindex = 6;
	else if( combinedAngle < 0)
 	stateindex = 5;
	else if(combinedAngle < 1)
 	stateindex = 4;
	else if( combinedAngle < 2)
 	stateindex = 3;
	else if( combinedAngle < 4 )
 	stateindex = 2;
        else if( combinedAngle < 8 )
        stateindex = 1;
        else 
        stateindex = 0;

	return stateindex;
}

int main(int argc, char **  argv){
	// initialize the ros system and node
	ros::init (argc, argv, "subodom");
	ros::NodeHandle n;
	ros::Rate rate(5);

	//Initialize variables
	int expRate = 0;//Ratio between exploration and implementation(1=explore, 0=implement)
	double reward= 0;
	int oldState=0;
	int  newState;
	double combinedOld = 0;
	double combinedNew = 0;
        double discount = 0.7;  //Higher means more discounting
        double learnRate = 0.5; //Higher is faster
	double action =0.0;
	double currentState= getState(combinedAngle);
	int thisAction =0;
	double qVals[10][9]={
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0},
				{0,0,0,0,0,0,0,0,0}
			};

	//subscribe to ROS nodes
	ros::Subscriber sub = n.subscribe ("/odom", 1000, &poseMsgReceived);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);// publish at a slower time

	//Load previously saved qVals matrix
	ifstream in;
	in.open("example.txt");
	if (!in){
		
		ROS_INFO_STREAM("0s");
	}
	else {
		// Read the saved Q matrix

		ROS_INFO_STREAM("from example.txt");
		for (int y = 0; y < 10; y++) {
			for (int x = 0; x < 9; x++) {
				in >> qVals[x][y];
			}
		}
	}

	//Main loop
	while (ros::ok() && (g_x) < 4.0)
	{
		//Sets speed
		geometry_msgs::Twist msg;
		msg.linear.x = 0.2;
		pub.publish(msg);
		
		//Choose action to take
		oldState = getState(combinedOld);
		if (expRate == 0 && (oldState==0 || oldState==9)) {
			double maxQ = -10000000;
			int maxQidx = 4;
			for (int actionNum=0;actionNum<=8;actionNum++){
				if (qVals[oldState][actionNum] > maxQ) {
					maxQidx = actionNum;
					maxQ = qVals[oldState][actionNum];
				}
			}
			thisAction = maxQidx;
		}
		else {
			thisAction = rand() % 8;
		}
		
		//Apply the action
		action = publishAction(thisAction); // converts action# to angVel
		msg.angular.z = action;  // publish angular value 
		pub.publish(msg); // apply angular action

		//Observe new state
		combinedNew = combinedAngle;
		reward = abs(combinedOld) -abs(combinedNew);    // if 0 then maybe combined angle hasn;t been updated 
		newState = getState(combinedNew);
		
		//Get optimal qVal of new state
		int maxQ = 0;
		for (int a = 0; a <9; a++)
		{
			int val = qVals[newState][a];
			maxQ = max(maxQ,val);
		}

		//Update qVal of old state
		qVals[oldState][thisAction] = (1 - learnRate)*qVals[oldState][thisAction] + learnRate*(reward + discount * maxQ);

		//Output into to terminal
		ROS_INFO_STREAM("combined angle:    "<<combinedNew);
		ROS_INFO_STREAM("linear x"<<  g_x);
		ROS_INFO_STREAM("linear y"<<  g_y);
		ROS_INFO_STREAM("reward  "<<reward);
		ROS_INFO_STREAM("action "<<  action);
		ROS_INFO_STREAM("action index "<<  thisAction);
		//ROS_INFO_STREAM("combined old "<<combinedOld);
		
		//Finish main loop
		ros::spinOnce();
		rate.sleep();
		combinedOld = combinedNew;
	}

	//Save updated qVal matrix to file
	fstream myfile;
	myfile.open("example.txt", fstream::out);
	for (int i = 0; i< 10; i++) {
		// Prints row of Q
		ROS_INFO_STREAM("eached file write");
		for (int j = 0; j<9; j++)
		{
			//Print each value
			myfile << qVals[i][j] << "\t";
		}
		myfile << std::endl;
	}
	myfile.close();

	return 0;
}
