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

#include <RobotModel.h>

#define ROOT NULL

using namespace icub::robot_model;

void RobotModel::calcInterference(Matrix &distance)
{
	distance.resize(selfDistance.R);

	if (selfDirty)
	{
		selfDirty = false;

		for (unsigned int c = 0; c < cover_list.size(); ++c)
		{
			cover_list[c]->pose();
		}

		Vec3 Xa, Xb, Ud;

		for (unsigned int i = 0; i < interference.size(); ++i)
		{
			selfDistance(i) = repulsion(interference[i]->coverA, interference[i]->coverB, Xa, Xb, Ud);
		}
	}

	distance = selfDistance;
}

const Matrix& RobotModel::calcInterferenceAndJ(Matrix &distance)
{
	distance.resize(selfDistance.R);

	if (selfDirty)
	{
		selfDirty = false;

		for (unsigned int c = 0; c < cover_list.size(); ++c)
		{
			cover_list[c]->pose();
		}

		Vec3 Xa, Xb, Ud;

		for (unsigned int i = 0; i < interference.size(); ++i)
		{
			selfDistance(i) = repulsion(interference[i]->coverA, interference[i]->coverB, Xa, Xb, Ud);

			Component *solidA = interference[i]->coverA->part;

			for (unsigned int d = 0; d < interference[i]->jdep.size(); ++d)
			{
				int j = interference[i]->jdep[d];

				Jself(i, j) = Ud * (solidA->Voj[j] + (solidA->Zoj[j] % (Xa - solidA->Poj[j])));
			}
		}
	}

	distance = selfDistance;

	return Jself;
}

