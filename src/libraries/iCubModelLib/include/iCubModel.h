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

#ifndef __R1_MODEL_H___
#define __R1_MODEL_H___

#include <RobotModel.h>

namespace icub
{
	namespace robot_model
	{
		namespace iCub
		{

			class iCubModel : public RobotModel
			{
			public:
				iCubModel();

				int getNDOF(){ return NJOINTS; }

				virtual const Matrix& calcGravity(Vec3 &com)
				{
					bool dirty = gravDirty;

					RobotModel::calcGravity(com);

					if (dirty)
					{
						Jgrav(0, LEFT_WRIST_ROT) = 0.0; Jgrav(0, LEFT_WRIST_PAN) = 0.0; Jgrav(0, LEFT_WRIST_TILT) = 0.0;
						Jgrav(1, LEFT_WRIST_ROT) = 0.0; Jgrav(1, LEFT_WRIST_PAN) = 0.0; Jgrav(1, LEFT_WRIST_TILT) = 0.0;

						Jgrav(0, RIGHT_WRIST_ROT) = 0.0; Jgrav(0, RIGHT_WRIST_PAN) = 0.0; Jgrav(0, RIGHT_WRIST_TILT) = 0.0;
						Jgrav(1, RIGHT_WRIST_ROT) = 0.0; Jgrav(1, RIGHT_WRIST_PAN) = 0.0; Jgrav(1, RIGHT_WRIST_TILT) = 0.0;

						Jgrav(0, HEAD_PITCH) = 0.0; Jgrav(0, HEAD_ROLL) = 0.0; Jgrav(0, HEAD_YAW) = 0.0;
						Jgrav(1, HEAD_PITCH) = 0.0; Jgrav(1, HEAD_ROLL) = 0.0; Jgrav(1, HEAD_YAW) = 0.0;
					}

					return Jgrav;
				}

				double getBalancing(Vec3 &Force)
				{
					Force.clear();

					return 100.0;
				}

			protected:
				enum
				{
					TORSO_PITCH,			
					TORSO_ROLL,				
					TORSO_YAW,				

					HEAD_PITCH,				
					HEAD_ROLL,				
					HEAD_YAW,				

					LEFT_SHOULDER_0,		
					LEFT_SHOULDER_1,		
					LEFT_SHOULDER_2,		
					LEFT_ELBOW,

					LEFT_WRIST_ROT,			
					LEFT_WRIST_PAN,			
					LEFT_WRIST_TILT,		

					RIGHT_SHOULDER_0,		
					RIGHT_SHOULDER_1,		
					RIGHT_SHOULDER_2,		
					RIGHT_ELBOW,			

					RIGHT_WRIST_ROT,		
					RIGHT_WRIST_PAN,		
					RIGHT_WRIST_TILT,		
				
					LEFT_HIP_0,				
					LEFT_HIP_1,				
					LEFT_HIP_2,				 
					LEFT_KNEE,				
					LEFT_ANKLE_PITCH,		
					LEFT_ANKLE_ROLL,		

					RIGHT_HIP_0,			
					RIGHT_HIP_1,			
					RIGHT_HIP_2,			
					RIGHT_KNEE,				
					RIGHT_ANKLE_PITCH,		
					RIGHT_ANKLE_ROLL,		

					NJOINTS
				};

				enum{ HIP, TORSO, LEFT_UPPER_ARM, LEFT_LOWER_ARM, LEFT_HAND, RIGHT_UPPER_ARM, RIGHT_LOWER_ARM, RIGHT_HAND, LEFT_UPPER_LEG, LEFT_LOWER_LEG, LEFT_FOOT, RIGHT_UPPER_LEG, RIGHT_LOWER_LEG, RIGHT_FOOT, HEAD, NPARTS };
			};
		}
	}
}

#endif