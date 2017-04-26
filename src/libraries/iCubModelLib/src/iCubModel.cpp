/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
* Author: Alessandro Scalzo
* email:  alessandro.scalzo@iit.it
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include <iCubModel.h>

#define JINIT(name) name, qmin(name), qmax(name), newjoint

#define JLIMIT(name, q0, q1) qmin(name) = q0; qmax(name) = q1

#define ROOT NULL

//#define STOP_DISTANCE 0.05

using namespace icub::robot_model::iCub;

iCubModel::iCubModel() : RobotModel()
{
	qmin.resize(NJOINTS);
	qmax.resize(NJOINTS);

	JLIMIT(TORSO_PITCH, -20, 70);	
	JLIMIT(TORSO_ROLL, -30, 30);	
	JLIMIT(TORSO_YAW, -50, 50);		

	JLIMIT(HEAD_PITCH, -30, 22);
	JLIMIT(HEAD_ROLL, -20, 20);
	JLIMIT(HEAD_YAW, -45, 45);

	JLIMIT(LEFT_SHOULDER_0, -95.5, 8);
	JLIMIT(LEFT_SHOULDER_1, 9, 160);	
	JLIMIT(LEFT_SHOULDER_2, -32, 80);	
	JLIMIT(LEFT_ELBOW, 5, 106);

	JLIMIT(LEFT_WRIST_ROT, -60, 60);	
	JLIMIT(LEFT_WRIST_PAN, -80, 25);	
	JLIMIT(LEFT_WRIST_TILT, -20, 25);	

	JLIMIT(RIGHT_SHOULDER_0, -95.5, 8);	
	JLIMIT(RIGHT_SHOULDER_1, 9, 160);	
	JLIMIT(RIGHT_SHOULDER_2, -32, 80);	
	JLIMIT(RIGHT_ELBOW, 5, 106);		

	JLIMIT(RIGHT_WRIST_ROT, -60, 60);
	JLIMIT(RIGHT_WRIST_PAN, -80, 25);
	JLIMIT(RIGHT_WRIST_TILT, -20, 25);

	JLIMIT(LEFT_HIP_0, -30, 85);		
	JLIMIT(LEFT_HIP_1, -12, 85);		
	JLIMIT(LEFT_HIP_2, -70, 70);		 
	JLIMIT(LEFT_KNEE, -100, 0);			
	JLIMIT(LEFT_ANKLE_PITCH, -30, 30);	
	JLIMIT(LEFT_ANKLE_ROLL, -20, 20);	

	JLIMIT(RIGHT_HIP_0, -30, 85);		
	JLIMIT(RIGHT_HIP_1, -12, 85);		
	JLIMIT(RIGHT_HIP_2, -70, 70);		
	JLIMIT(RIGHT_KNEE, -100, 0);		
	JLIMIT(RIGHT_ANKLE_PITCH, -30, 30);	
	JLIMIT(RIGHT_ANKLE_ROLL, -20, 20);		

	Component *newjoint;

	T_ROOT.Pj().z = 0.5975;

	mRoot = newjoint = new Link(T_ROOT, ROOT);

	// upper body
	newjoint = new Link(Transform(90, -90, 0, 0, 0, 0), newjoint);

	newjoint = new RotJoint(JINIT(TORSO_PITCH));

	newjoint = new Link(Transform(0.032, 0, 90, 0), newjoint);
	newjoint->setGlocal(0.779, 0.031, 0, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(TORSO_ROLL));

	newjoint = new Link(Transform(0, -0.0055, 90, -90), newjoint);
	newjoint->setGlocal(2.0*0.577, 0, 0, -0.001);
	heavy_part.push_back(newjoint);

	Component* torsoYaw = newjoint = new RotJoint(JINIT(TORSO_YAW));

	newjoint = new Link(Transform(0.00231, -0.1933, - 90, -90), newjoint);
	newjoint->setGlocal(4.81, 0, -0.118, 0.008);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(HEAD_PITCH));

	newjoint = new Link(Transform(0.033, 0, 90, 90), newjoint);
	newjoint->setGlocal(0.270, -0.030, 0, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(HEAD_ROLL));

	newjoint = new Link(Transform(0, 0.001, -90, -90), newjoint);
	newjoint->setGlocal(0.272, 0, 0.004, 0.005);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(HEAD_YAW));

	Component* head = newjoint = new Link(Transform(-0.054, 0.0825, -90, 90), newjoint);
	newjoint->setGlocal(1.336, 0, 0, 0);
	heavy_part.push_back(newjoint);

	// left arm

	newjoint = new Link(Transform(0.0233647, -0.1433, -90, 105), torsoYaw);

	newjoint = new RotJoint(JINIT(LEFT_SHOULDER_0));

	newjoint = new Link(Transform(0, 0.10774, -90, 90), newjoint);
	newjoint->setGlocal(0.189, -0.005, 0.018, -0.001);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_SHOULDER_1));

	newjoint = new Link(Transform(0, 0, 90, -90), newjoint);
	newjoint->setGlocal(0.179, 0, -0.006, 0.016);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_SHOULDER_2));

	newjoint = new Link(Transform(0.015, 0.15228, -90, 75), newjoint);
	newjoint->setGlocal(0.884, -0.001, 0.063, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_ELBOW));

	newjoint = new Link(Transform(-0.015, 0, 90, 0), newjoint);
	newjoint->setGlocal(0.074, 0.013, 0.003, -0.001);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_WRIST_ROT));

	newjoint = new Link(Transform(0, 0.1373, 90, -90), newjoint);
	newjoint->setGlocal(0.525, 0, -0.071, 0.004);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_WRIST_PAN));

	newjoint = new Link(Transform(0, 0, 90, 90), newjoint);

	newjoint = new RotJoint(JINIT(LEFT_WRIST_TILT));

	mHand[L] = newjoint = new Link(Transform(0.0625, -0.016, 0, 0), newjoint);
	newjoint->setGlocal(0.213, 0.007, -0.008, 0.009);
	heavy_part.push_back(newjoint);

	// right arm

	newjoint = new Link(Transform(-0.0233647, -0.1433, 90, -105), torsoYaw);

	newjoint = new RotJoint(JINIT(RIGHT_SHOULDER_0));

	newjoint = new Link(Transform(0, -0.10774, 90, -90), newjoint);
	newjoint->setGlocal(0.189, 0, 0.018, 0.001);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_SHOULDER_1));

	newjoint = new Link(Transform(0, 0, -90, -90), newjoint);
	newjoint->setGlocal(0.179, 0, -0.006, -0.016);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_SHOULDER_2));

	newjoint = new Link(Transform(-0.015, -0.15228, -90, -105), newjoint);
	newjoint->setGlocal(0.884, 0.001, -0.063, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_ELBOW));

	newjoint = new Link(Transform(0.015, 0, 90, 0), newjoint);
	newjoint->setGlocal(0.074, -0.013, -0.003, 0.001);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_WRIST_ROT));

	newjoint = new Link(Transform(0, -0.1373, 90, -90), newjoint);
	newjoint->setGlocal(0.525, 0, 0.071, -0.004);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_WRIST_PAN));

	newjoint = new Link(Transform(0, 0, 90, 90), newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_WRIST_TILT));

	mHand[R] = newjoint = new Link(Transform(0.0625, 0.016, 0, 180), newjoint);
	newjoint->setGlocal(0.213, 0.007, -0.008, -0.009);
	heavy_part.push_back(newjoint);

	// left leg

	newjoint = new Link(Transform(0, 0, -90, 0, -0.0681, -0.1199), mRoot);

	newjoint = new RotJoint(JINIT(LEFT_HIP_0));

	newjoint = new Link(Transform(0, 0, -90, 90), newjoint);
	newjoint->setGlocal(0.754, 0, 0, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_HIP_1));

	newjoint = new Link(Transform(0, 0, -90, 90), newjoint);
	newjoint->setGlocal(0.526, 0, 0, -0.030);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_HIP_2));

	newjoint = new Link(Transform(0, -0.2236, 90, -90), newjoint);
	newjoint->setGlocal(2.175, 0.001, 0.064, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_KNEE));

	newjoint = new Link(Transform(-0.213, 0, 180, 90), newjoint);
	newjoint->setGlocal(1.264, 0.106, 0.001, 0.002);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_ANKLE_PITCH));

	newjoint = new Link(Transform(0, 0, -90, 0), newjoint);
	newjoint->setGlocal(0.746, -0.005, 0.001, 0.017);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(LEFT_ANKLE_ROLL));

	newjoint = new Link(Transform(-0.041, 0, 0, 0), newjoint);
	newjoint->setGlocal(0.861, 0.032, 0, 0.024);
	heavy_part.push_back(newjoint);

	// right leg

	newjoint = new Link(Transform(0, 0, -90, 0, 0.0681, -0.1199), mRoot);

	newjoint = new RotJoint(JINIT(RIGHT_HIP_0));

	newjoint = new Link(Transform(0, 0, 90, 90), newjoint);
	newjoint->setGlocal(0.754, 0, -0.078, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_HIP_1));

	newjoint = new Link(Transform(0, 0, 90, 90), newjoint);
	newjoint->setGlocal(0.526, 0, 0, 0.030);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_HIP_2));

	newjoint = new Link(Transform(0, 0.2236, -90, -90), newjoint);
	newjoint->setGlocal(2.175, 0.001, 0.064, 0);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_KNEE));

	newjoint = new Link(Transform(-0.213, 0, 180, 90), newjoint);
	newjoint->setGlocal(1.264, 0.105, 0.001, -0.002);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_ANKLE_PITCH));

	newjoint = new Link(Transform(0, 0, 90, 0), newjoint);
	newjoint->setGlocal(0.746, -0.005, 0.001, -0.017);
	heavy_part.push_back(newjoint);

	newjoint = new RotJoint(JINIT(RIGHT_ANKLE_ROLL));

	newjoint = new Link(Transform(-0.041, 0, 180, 0), newjoint);
	newjoint->setGlocal(0.861, 0.032, 0, 0.024);
	heavy_part.push_back(newjoint);

	///////////////////////////////////////////////////////////////////////////////////

	Matrix q0(NJOINTS);

	// masses

	mRoot->setPoseCalcJ(q0, T_ROOT);

	// covers

	Cover *cover[NPARTS];

	//cover[HIP] = NULL;
	//cover[TORSO] = new Cover(TORSO);
	//cover[LEFT_UPPER_ARM] = new Cover(LEFT_UPPER_ARM);
	//cover[LEFT_LOWER_ARM] = new Cover(LEFT_LOWER_ARM);
	//cover[LEFT_HAND] = new Cover(LEFT_HAND);
	//cover[RIGHT_UPPER_ARM] = new Cover(RIGHT_UPPER_ARM);
	//cover[RIGHT_LOWER_ARM] = new Cover(RIGHT_LOWER_ARM);
	//cover[RIGHT_HAND] = new Cover(RIGHT_HAND);
	//cover[HEAD] = NULL;

	//for (int p = 0; p < NPARTS; ++p) if (cover[p]) cover_list.push_back(cover[p]);

#define STORE_SPHERE sphere_list.push_back(NULL); sphere_list.back() = 

#if 0
	STORE_SPHERE cover[TORSO]->addSphere(0.05, 0.0, 0.03, 0.08, "3_0");
	STORE_SPHERE cover[TORSO]->addSphere(-0.04, -0.06, 0.03, 0.08, "3_1");
	STORE_SPHERE cover[TORSO]->addSphere(-0.04, 0.06, 0.03, 0.08, "3_2");
	STORE_SPHERE cover[TORSO]->addSphere(0.08, 0.0, 0.12, 0.09, "3_3");
	STORE_SPHERE cover[TORSO]->addSphere(0.01, -0.06, 0.12, 0.09, "3_4");
	STORE_SPHERE cover[TORSO]->addSphere(0.01, 0.06, 0.12, 0.09, "3_5");
	STORE_SPHERE cover[TORSO]->addSphere(0.1, 0.0, 0.21, 0.09, "3_6");
	STORE_SPHERE cover[TORSO]->addSphere(0.03, -0.07, 0.21, 0.09, "3_7");
	STORE_SPHERE cover[TORSO]->addSphere(0.03, 0.07, 0.21, 0.09, "3_8");
	STORE_SPHERE cover[TORSO]->addSphere(0.12, 0.0, 0.3, 0.08, "3_9");
	STORE_SPHERE cover[TORSO]->addSphere(0.06, -0.1, 0.32, 0.07, "3_10");
	STORE_SPHERE cover[TORSO]->addSphere(0.06, 0.1, 0.32, 0.07, "3_11");
	STORE_SPHERE cover[TORSO]->addSphere(0.06, 0.0, 0.32, 0.07, "3_12");
	STORE_SPHERE cover[TORSO]->addSphere(0.08, -0.19, 0.37, 0.06, "3_13");
	STORE_SPHERE cover[TORSO]->addSphere(0.08, 0.19, 0.37, 0.06, "3_14");

	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.0, 0.0, 0.035, "6_0");
	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.06, 0.0, 0.045, "6_1");
	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.12, 0.0, 0.045, "6_2");
	STORE_SPHERE cover[LEFT_UPPER_ARM]->addSphere(0.0, 0.18, 0.0, 0.045, "6_3");

	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, 0.0, 0.0375, "8_0");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.04, 0.04, "8_1");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.10, 0.0425, "8_2");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.16, 0.045, "8_3");
	STORE_SPHERE cover[LEFT_LOWER_ARM]->addSphere(0.0, 0.0, -0.22, 0.0425, "8_4");

	STORE_SPHERE cover[LEFT_HAND]->addSphere(0.0, 0.0, 0.02, 0.02, "11_0");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.04, 0.0, 0.01, 0.03, "11_1");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.08, 0.0, 0.02, 0.02, "11_2");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.02, 0.0, -0.02, 0.02, "11_3");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(0.02, 0.0, 0.02, 0.016, "11_4");
	STORE_SPHERE cover[LEFT_HAND]->addSphere(-0.01, 0.0, -0.03, 0.016, "11_5");

	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.0, 0.0, 0.035, "14_0");
	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.06, 0.0, 0.045, "14_1");
	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.12, 0.0, 0.045, "14_2");
	STORE_SPHERE cover[RIGHT_UPPER_ARM]->addSphere(0.0, 0.18, 0.0, 0.045, "14_3");

	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, 0.0, 0.0375, "16_0");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.04, 0.04, "16_1");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.10, 0.0425, "16_2");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.16, 0.045, "16_3");
	STORE_SPHERE cover[RIGHT_LOWER_ARM]->addSphere(0.0, 0.0, -0.22, 0.0425, "16_4");

	STORE_SPHERE cover[RIGHT_HAND]->addSphere(0.0, 0.0, -0.02, 0.02, "19_0");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.04, 0.0, -0.01, 0.03, "19_1");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.08, 0.0, -0.02, 0.02, "19_2");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.02, 0.0, 0.02, 0.02, "19_3");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(0.02, 0.0,  -0.02, 0.016, "19_4");
	STORE_SPHERE cover[RIGHT_HAND]->addSphere(-0.01, 0.0, 0.03, 0.016, "19_5");

	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[TORSO], 4, 7));
	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[RIGHT_UPPER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[RIGHT_LOWER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_LOWER_ARM], cover[RIGHT_HAND], 4, 7));

	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[TORSO], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[LEFT_UPPER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[LEFT_LOWER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_LOWER_ARM], cover[LEFT_HAND], 12, 15));

	interference.push_back(new Interference(cover[LEFT_HAND], cover[TORSO], 4, 7));
	interference.push_back(new Interference(cover[LEFT_HAND], cover[RIGHT_UPPER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_HAND], cover[RIGHT_LOWER_ARM], 4, 7));
	interference.push_back(new Interference(cover[LEFT_HAND], cover[RIGHT_HAND], 4, 7));

	interference.push_back(new Interference(cover[RIGHT_HAND], cover[TORSO], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_HAND], cover[LEFT_UPPER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_HAND], cover[LEFT_LOWER_ARM], 12, 15));
	interference.push_back(new Interference(cover[RIGHT_HAND], cover[LEFT_HAND], 12, 15));

	Jself.resize(interference.size(), NJOINTS);
#endif

	Jgrav.resize(2, NJOINTS);

	Jhand[0].resize(6, NJOINTS);
	Jhand[1].resize(6, NJOINTS);

//	selfDistance.resize(interference.size());
}
