#include "lab4pkg/lab4.h"
#include <math.h>

/** 
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{

	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end; 
	double xgrip,ygrip,zgrip;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;
	double yaw, pi;
	double l1, l2, l3, l4;
	double t2_star, t3_star, t2_cross;

	pi = 3.14159265;
	a1 = 0;
	d1 = 0.152;
	a2 = 0.244;
	d2 = 0.120;
	a3 = 0.213;
	d3 = -0.093;
	a4 = 0;
	d4 = 0.083;
	a5 = 0;
	d5 = 0.083;
	a6 = 0.0535;
	//d6 = (0.082+0.056);
	d6 = (0.148);

	xgrip = xWgrip;
	ygrip = yWgrip;
	zgrip = zWgrip;
	yaw = (pi * yaw_WgripDegree)/180.0;

	zcen=zWgrip;
	xcen=xWgrip-a6*cos(yaw);
	ycen=yWgrip-a6*sin(yaw);

	ROS_INFO("(x,y,z) cen: %f, %f, %f", xcen, ycen, zcen);

	double theta1p,theta1pp;
	theta1p=atan2(ycen,xcen);
	theta1pp=asin(.110/sqrt(xcen*xcen+ycen*ycen));
	theta1=theta1p-theta1pp;
	//if (theta1 > PI) theta1 -= PI;
	ROS_INFO("theta1pp inside: %f", .110/sqrt(xcen*xcen+ycen*ycen));
	ROS_INFO("(theta1p,theta1pp,theta1): %f, %f, %f", theta1p, theta1pp, theta1);

	theta6 = (yaw - theta1 - PI/2)*-1;

	z3end = zcen + d6;
	y3end = ycen - d5*sin(theta1) - 0.110*cos(theta1);
	x3end = xcen - d5*cos(theta1) + 0.110*sin(theta1);
	ROS_INFO("(x,y,z) end: %f, %f, %f", x3end, y3end, z3end);

	l1 = sqrt(x3end*x3end + y3end*y3end);
	l2 = z3end - d1;
	l3 = sqrt(l1*l1 +l2*l2);

	ROS_INFO("(l1,l2,l3): %f, %f, %f", l1,l2,l3);


	double theta2p, theta2pp, theta3p;

	theta2p = asin(l2/l3);
	theta2pp = acos((l3*l3 + a2*a2 - a3*a3)/(2*l3*a2));
	theta2 = -1*(theta2p + theta2pp);
	ROS_INFO("(theta2p, theta2pp, theta2): %f, %f, %f", theta2p,theta2pp,theta2);

	theta3p = acos((a2*a2 + a3*a3 - l3*l3)/(2*a2*a3));
	theta3 = PI - theta3p;

	theta4 = -PI/2 - theta3 - theta2;
	theta5 = -PI/2;
	
	// View values
	//use cout
	cout<<"theta1: "<< theta1<<endl;
	cout<<"theta2: "<< theta2<<endl;
	cout<<"theta3: "<< theta3<<endl;
	cout<<"theta4: "<< theta4<<endl;
	cout<<"theta5: "<< theta5<<endl;
	cout<<"theta6: "<< theta6<<endl;

	// check that your values are good BEFORE sending commands to UR3
	//lab_fk calculates the forward kinematics and convert it to std::vector<double>
	return lab_fk((float)theta1,(float)theta2,(float)theta3,(float)theta4,(float)theta5,(float)theta6);
}

