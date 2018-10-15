#include "lab4pkg/lab4.h"

/** 
 * function that calculates an elbow up Inverse Kinematic solution for the UR3
 */
std::vector<double> lab_invk(float xWgrip, float yWgrip, float zWgrip, float yaw_WgripDegree)
{

	double xcen,ycen,zcen,theta6,theta5,theta4,theta3,theta2,theta1,x3end,y3end,z3end; 
	double xgrip,ygrip,zgrip;
	double a1,a2,a3,a4,a5,a6;
	double d1,d2,d3,d4,d5,d6;
	double yaw;
	double l1, l2, l3, l4;
	double t2_star t3_star t2_cross;

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
	d6 = (0.082+0.056);

	xgrip = xWgrip;
	ygrip = yWgrip;
	zgrip = zWgrip;
	yaw = yaw_WgripDegree;

	xcen = xgrip - a6*cos(yaw);
	ycen = ygrip - a6*sin(yaw);
	zcen = zgrip;

	l1 = sqrt(xcen^2 +ycen^2);
	l2 = sqrt(l1^2 - 110^2);

	
	theta1 = arctan2(xcen, ycen) - arctan(110.0/l2);
	theta6 = yaw - theta1;
 
	l3 = sqrt(13778 - 110^2);
	l4 = l2 - l3;

	x3end = l4*cos(theta1);
	y3end = l4*sin(theata1);
	z3end = zcen + .158; //using measurement from last lab

	
	t2_star=atan((z3end-d1)/l4)
	t3_star=acos((a2^2+a3^2-l4^2)/(2*a2*a3)
	t2_cross=acos((a2^2+l4^2-a3^2)/(2*a2*l4))

	theta2= t2_cross+t2_star; // Default value Need to Change
	theta3= PI-t3_star; // Default value Need to Change
	theta4= -theta2-theta3-PI/2  // Default value Need to Change
	theta5=-PI/2;  // Default value Need to Change
	



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
