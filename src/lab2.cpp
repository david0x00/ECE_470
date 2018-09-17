#include "lab2pkg/lab2.h" 
#define PI 3.14159265359
#define SPIN_RATE 20  /* Hz */

//arrays defining Waypoints
double home[]={156*PI/180,-90*PI/180,126*PI/180,-124*PI/180,-90*PI/180,38*PI/180};

double arr11[]={151*PI/180,-50*PI/180,103*PI/180,-143*PI/180,-90*PI/180,32*PI/180};
double arr12[]={151*PI/180,-56*PI/180,102*PI/180,-136*PI/180,-90*PI/180,33*PI/180};
double arr13[]={151*PI/180,-61*PI/180,99*PI/180,-129*PI/180,-90*PI/180,32*PI/180};
double arr14[]={151*PI/180,-69*PI/180,92*PI/180,-114*PI/180,-90*PI/180,32*PI/180};

double arr21[]={151*PI/180,-50*PI/180,103*PI/180,-143*PI/180,-90*PI/180,32*PI/180};
double arr22[]={120*PI/180,-64*PI/180,123*PI/180,-148*PI/180,-90*PI/180,0*PI/180};
double arr23[]={120*PI/180,-72*PI/180,120*PI/180,-137*PI/180,-90*PI/180,0*PI/180};
double arr24[]={120*PI/180,-78*PI/180,115*PI/180,-125*PI/180,-90*PI/180,0*PI/180};

double arr31[]={151*PI/180,-50*PI/180,103*PI/180,-143*PI/180,-90*PI/180,32*PI/180};
double arr32[]={120*PI/180,-64*PI/180,123*PI/180,-148*PI/180,-90*PI/180,0*PI/180};
double arr33[]={120*PI/180,-72*PI/180,120*PI/180,-137*PI/180,-90*PI/180,0*PI/180};
double arr34[]={120*PI/180,-78*PI/180,115*PI/180,-125*PI/180,-90*PI/180,0*PI/180};

// array to define final velocity of point to point moves.  For now slow down to zero once 
// each point is reached
double arrv[]={0,0,0,0,0,0};

//vectors to be used to publish commands to UR3 ROS Driver (ece470_ur3_driver)
std::vector<double> QH (home,home+sizeof(home) / sizeof(home[0]));

std::vector<double> Q11 (arr11,arr11+sizeof(arr11) / sizeof(arr11[0]));
std::vector<double> Q12 (arr12,arr12+sizeof(arr12) / sizeof(arr12[0]));
std::vector<double> Q13 (arr13,arr13+sizeof(arr13) / sizeof(arr13[0]));
std::vector<double> Q14 (arr14,arr14+sizeof(arr14) / sizeof(arr14[0]));

std::vector<double> Q21 (arr21,arr21+sizeof(arr21) / sizeof(arr21[0]));
std::vector<double> Q22 (arr22,arr22+sizeof(arr22) / sizeof(arr22[0]));
std::vector<double> Q23 (arr23,arr23+sizeof(arr23) / sizeof(arr23[0]));
std::vector<double> Q24 (arr24,arr24+sizeof(arr24) / sizeof(arr24[0]));

std::vector<double> Q31 (arr31,arr31+sizeof(arr31) / sizeof(arr31[0]));
std::vector<double> Q32 (arr32,arr32+sizeof(arr32) / sizeof(arr32[0]));
std::vector<double> Q33 (arr33,arr33+sizeof(arr33) / sizeof(arr33[0]));
std::vector<double> Q34 (arr34,arr34+sizeof(arr34) / sizeof(arr34[0]));

std::vector<double> v (arrv,arrv+sizeof(arrv) / sizeof(arrv[0]));

// creating an array of these vectors allows us to iterate through them
// and programatically choose where to go.
std::vector<double> Q [3][4] = {
    {Q11, Q12, Q13, Q14},
    {Q21, Q22, Q23, Q24},
    {Q31, Q32, Q33, Q34}
};


// Global bool variables that are assigned in the callback associated when subscribed 
// to the "ur3/position" topic
bool isReady=1;
bool pending=0;

// Whenever ur3/position publishes info this callback function is run.
void position_callback(const ece470_ur3_driver::positions::ConstPtr& msg)
{
	isReady=msg->isReady; // When isReady is True the robot arm has made it to its desired position
						  // and is ready to be told to go to another point if desired.
	pending=msg->pending; // pending is the opposite of isReady, pending is true until a new position is reached
//	ROS_INFO("Debug isRdy = %d, pending = %d",(int)isReady,(int)pending);
}

class TOH {
private:
    int[] heights = {0,0,0};
    int start_rod;
    int end_rod;
    int aux_rod;
    int num_blocks = 3;
    ros::Publisher pub_command;
    ros::Rate loop_rate;
    ros::ServiceClient srv_SetIO;
    ur_msgs::SetIO srv;

public:
    TOH(int n, char from_rod, char to_rod, char aux_rod,
        ros::Publisher pub_command, ros::Rate loop_rate,
        ros::ServiceClient srv_SetIO, ur_msgs::SetIO srv) {
        num_blocks = n;
        start_rod = from_rod;
        end_rod = to_rod;
        this->aux_rod = aux_rod;
        this->pub_command = pub_command;
        this->loop_rate = loop_rate;
        this->srv_SetIO = srv_SetIO;
        this->srv = srv;
        heights[start_rod] = num_blocks;
        heights[end_rod] = 0;
        heights[aux_rod] = 0;
        std_duration = 2.0;
    }

    void solve(int n, char from_rod, char to_rod, char aux_rod)
    {
        if (n == 1)
        {
            std::cout << "\n Move disk 1 from rod " << from_rod << " to rod " << to_rod << std::endl;
            move_block(pub_command, loop_rate, srv_SetIO, srv, from_rod, heights[from_rod], to_rod, heights[to_rod]);
            return;
        }
        solve(n-1, from_rod, aux_rod, to_rod);
        printf("\n Move disk %d from rod %c to rod %c", n, from_rod, to_rod);
        move_block(pub_command, loop_rate, srv_SetIO, srv, from_rod, heights[from_rod], to_rod, heights[to_rod]);
        solve(n-1, aux_rod, to_rod, from_rod);
    }

    int move_block(ros::Publisher pub_command ,
                    ros::Rate loop_rate,
                    ros::ServiceClient srv_SetIO,
                    ur_msgs::SetIO srv,
                    int start_loc,
                    int start_height,
                    int end_loc,
                    int end_height)
    {
        move_arm(pub_command, loop_rate, Q[start_loc][4], std_duration);
        move_arm(pub_command, loop_rate, Q[start_loc][start_height], std_duration);
        //turn suction on
        //test digital input 1
        int error = 0;
        return error;
    }

    int move_arm(ros::Publisher pub_command , ros::Rate loop_rate, std::vector<double> dest, float duration)
    {
        int error = 0;
        return error;
    }
}



int main(int argc, char **argv)
{
    
        int start_location = 0;
        int end_location = 0;
//initialization & variable definition
	ros::init(argc, argv, "lab2node");	//initialzation of ros required for each Node.
	ros::NodeHandle nh;				//handler for this node.
	
	//initialized publisher ur3/command, buffer size of 10.
	ros::Publisher pub_command=nh.advertise<ece470_ur3_driver::command>("ur3/command",10);
	// initialize subscriber to ur3/position and call function position_callback each time data is published
	ros::Subscriber sub_position=nh.subscribe("ur3/position",1,position_callback);
	
	ros::ServiceClient srv_SetIO = nh.serviceClient<ur_msgs::SetIO>("ur_driver/set_io");
	ur_msgs::SetIO srv;

	ece470_ur3_driver::command driver_msg;

        std::string inputString;
        while (!start_location && !end_location) {
                std::cout << "Choose starting location <Either 1 2 or 3>";
		std::getline(std::cin, inputString);
		std::cout << "You entered " << inputString << "\n";
		if (inputString == "1") {
                        start_location = 1;
		} else if (inputString == "2") {
                        start_location = 2;
		} else if (inputString == "3") {
                        start_location = 3;
		} else {
			std:cout << "Please just enter the character 1 2 or 3\n\n";
		}
                std::cout << "Choose ending location <Either 1 2 or 3>";
                std::getline(std::cin, inputString);
                std::cout << "You entered " << inputString << "\n";
                if (inputString == "1") {
                        end_location = 1;
                } else if (inputString == "2") {
                        end_location = 2;
                } else if (inputString == "3") {
                        end_location = 3;
                } else {
                        std:cout << "Please just enter the character 1 2 or 3\n\n";
                }
                if (start_location == end_location){
                    start_location = 0;
                    end_location = 0;
                    std::cout << "start and end have to be different" << std::endl;
                }
	}

	while(!ros::ok()){};	//check if ros is ready for operation
		
	ROS_INFO("sending Goals");

	ros::Rate loop_rate(SPIN_RATE); // Initialize the rate to publish to ur3/command
	int spincount = 0;
 
	while(Loopcnt > 0) {
                driver_msg.destination=QH;  // Set desired position to move home
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		


		ROS_INFO("sending Goals 1");
		driver_msg.destination=Q[0][0];
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		srv.request.fun = 1;
		srv.request.pin = 0;  //Digital Output 0
		srv.request.state = 1.0; //Set DO0 on
		if (srv_SetIO.call(srv)) {
			ROS_INFO("True: Switched Suction ON");
		} else {
			ROS_INFO("False");
		}

		ROS_INFO("sending Goals 2");
		driver_msg.destination=Q[0][1];
		driver_msg.duration=2.0;
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		srv.request.fun = 1;
		srv.request.pin = 0; // Digital Output 0
		srv.request.state = 0.0; //Set DO0 off
		if (srv_SetIO.call(srv)) {
			ROS_INFO("True: Switched Suction OFF");
		} else {
			ROS_INFO("False");
		}

		ROS_INFO("sending Goals 3");
		driver_msg.destination=Q[0][2];
		driver_msg.duration=1.0;
		pub_command.publish(driver_msg);  // publish command, but note that is possible that
												  // the subscriber will not receive this message.
		spincount = 0;
		while (isReady) { // Waiting for isReady to be false meaning that the driver has the new command
			ros::spinOnce();  // Allow other ROS functionallity to run
			loop_rate.sleep(); // Sleep and wake up at 1/20 second (1/SPIN_RATE) interval
			if (spincount > SPIN_RATE) {  // if isReady does not get set within 1 second re-publish
				pub_command.publish(driver_msg);
				ROS_INFO("Just Published again driver_msg");
				spincount = 0;
			}
			spincount++;  // keep track of loop count
		}

		ROS_INFO("waiting for rdy");  // Now wait for robot arm to reach the commanded waypoint.
		while(!isReady)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		Loopcnt--;
	}

	return 0;
}
