#ifndef __SAFE_MOTION_CONTROL_H__
#define __SAFE_MOTION_CONTROL_H__

#include <yarp/os/RateThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/ISafeControl.h>

#include <yarp/dev/ControlBoardInterfaces.h>

#include <RobotModel.h>
#include <R1Model.h>

#include <Matrix.h>

using namespace icub::robot_model;

#define MARGIN 0.1
#define PERIOD 0.01

class SafeMotionControl : 
	public yarp::os::RateThread, 
	public yarp::dev::DeviceDriver,
	public yarp::dev::IMultipleWrapper,
    public yarp::dev::ISafeControl
{
public:
	SafeMotionControl() : RateThread(int(PERIOD*1000.0))
	{
        robotModel = new icub::robot_model::r1::R1Model();
	}

    virtual ~SafeMotionControl()
    {
        if (robotModel) delete robotModel;
    }

	void run()
	{
		static Matrix dQ, dD;

        getPos(mQ);
		
		robotModel->setPose(mQ);
		robotModel->calcInterference(mD);

		bool safe = true;

		for (int i = 0; i < mD.R; ++i)
		{
			if (mD(i) < MARGIN) safe = false;
		}

		if (safe) return;

        getVel(mV);

		dQ = mQ + mV*PERIOD;

		robotModel->setPose(dQ);
		robotModel->calcInterference(dD);

		safe = true;

		for (int i = 0; i < mD.R; ++i)
		{
			if (dD(i) < MARGIN && dD(i) < mD(i))
			{
				safe = false;
			}
		}

        if (!safe)
        {
            static unsigned int n = 0;
            static int noflood = 0;

            if (++noflood >= 50)
            {
                noflood = 0;
                printf("\nUNSAFE %d\n\n", ++n);
            }
        }
	}

	bool checkVelocity(double *v)
	{
		static Matrix dQ, dD;

        getPos(mQ);

        robotModel->setPose(mQ);
        robotModel->calcInterference(mD);

        dQ.resize(mQ.R);

        for (int i = 0; i < mQ.R; ++i) dQ(i) = mQ(i) + v[i]*PERIOD;

		robotModel->setPose(dQ);
		robotModel->calcInterference(dD);

		bool safe = true;

		for (int i = 0; i < dD.R; ++i)
		{
			if (dD(i) <= MARGIN && dD(i) <= mD(i))
			{
				safe = false;
			}
		}

		if (safe) return true;

		for (int j = 0; j < NDOF; ++j)
		{
			dQ = mQ;

			dQ(j) += v[j]*PERIOD;

			robotModel->setPose(dQ);
			robotModel->calcInterference(dD);

			for (int i = 0; i < dD.R; ++i)
			{
				if (dD(i) <= MARGIN && dD(i) <= mD(i))
				{
					v[j] = 0.0;

					break;
				}
			}
		}

		return false;
	}

	bool checkPosition(double *p)
	{
		static Matrix dQ, dD;

        getPos(mQ);

        robotModel->setPose(mQ);
        robotModel->calcInterference(mD);

        dQ.resize(mQ.R);

        for (int i = 0; i < mQ.R; ++i) dQ(i) = p[i];

		robotModel->setPose(dQ);
		robotModel->calcInterference(dD);

		bool safe = true;

		for (int i = 0; i < dD.R; ++i)
		{
			if (dD(i) <= MARGIN && dD(i) <= mD(i))
			{
				safe = false;
			}
		}

		if (safe) return true;

		for (int j = 0; j < NDOF; ++j)
		{
			dQ = mQ;

			dQ(j) = p[j];

			robotModel->setPose(dQ);

			robotModel->calcInterference(dD);

			for (int i = 0; i < dD.R; ++i)
			{
				if (dD(i) <= MARGIN && dD(i) <= mD(i))
				{
					p[j] = mQ(j);

					break;
				}
			}
		}

		return false;
	}

	bool attachAll(const yarp::dev::PolyDriverList &p)
	{
		for (int i = 0; i < p.size(); ++i)
		{
            if (!p[i]->poly->isValid()) return false;

            int part;

            if (p[i]->key == "torso") part = TORSO;
            else if (p[i]->key == "head") part = HEAD;
            else if (p[i]->key == "left_upper_arm") part = LEFT_UPPER_ARM;
            else if (p[i]->key == "left_lower_arm") part = LEFT_LOWER_ARM;
            else if (p[i]->key == "right_upper_arm") part = RIGHT_UPPER_ARM;
            else if (p[i]->key == "right_lower_arm") part = RIGHT_LOWER_ARM;
            else return false;

            p[i]->poly->view(pEncFbk[part]);

            if (!pEncFbk[part]) return false;

            p[i]->poly->view(pPosCtrl[part]);

            if (!pPosCtrl[part]) return false;

            p[i]->poly->view(pVelCtrl[part]);
            
            if (!pVelCtrl[part]) return false;

            p[i]->poly->view(pDirCtrl[part]);

            if (!pDirCtrl[part]) return false;

            p[i]->poly->view(pCtrlMode[part]);

            if (!pCtrlMode[part]) return false;
		}

        start();

		return true;
	}

	bool detachAll()
	{
        if (isRunning()) stop();

		return true;
	}

protected:
    enum { TORSO, HEAD, LEFT_UPPER_ARM, LEFT_LOWER_ARM, RIGHT_UPPER_ARM, RIGHT_LOWER_ARM, N_ROBOT_PARTS };

    yarp::dev::IEncoders         *pEncFbk[N_ROBOT_PARTS];

    yarp::dev::IPositionControl2  *pPosCtrl[N_ROBOT_PARTS];
    yarp::dev::IVelocityControl2  *pVelCtrl[N_ROBOT_PARTS];
    yarp::dev::IPositionDirect    *pDirCtrl[N_ROBOT_PARTS];

    yarp::dev::IControlMode2      *pCtrlMode[N_ROBOT_PARTS];

	Matrix mQ;
	Matrix mV;
	Matrix mD;

	int NDOF;

	icub::robot_model::RobotModel* robotModel;


    virtual void getPos(Matrix& q)
    {
        double enc[8];

        pEncFbk[TORSO]->getEncoders(enc);

        q(0) = enc[0]; q(1) = enc[1]; q(2) = enc[2]; q(3) = enc[3];

        pEncFbk[LEFT_UPPER_ARM]->getEncoders(enc);

        q(4) = enc[0]; q(5) = enc[1]; q(6) = enc[2]; q(7) = enc[3]; 
        
        pEncFbk[LEFT_LOWER_ARM]->getEncoders(enc);

        q(8) = enc[0]; q(9) = enc[1]; q(10) = enc[2]; q(11) = enc[3];

        pEncFbk[RIGHT_UPPER_ARM]->getEncoders(enc);

        q(12) = enc[0]; q(13) = enc[1]; q(14) = enc[2]; q(15) = enc[3];

        pEncFbk[RIGHT_LOWER_ARM]->getEncoders(enc);
        
        q(16) = enc[0]; q(17) = enc[1]; q(18) = enc[2]; q(19) = enc[3];

        pEncFbk[HEAD]->getEncoders(enc);

        q(20) = enc[0]; q(21) = enc[1];
    }

    virtual void getVel(Matrix& v)
    {
        double enc[8];

        pEncFbk[TORSO]->getEncoderSpeeds(enc);

        v(0) = enc[0]; v(1) = enc[1]; v(2) = enc[2]; v(3) = enc[3];

        pEncFbk[LEFT_UPPER_ARM]->getEncoderSpeeds(enc);

        v(4) = enc[0]; v(5) = enc[1]; v(6) = enc[2]; v(7) = enc[3]; 
        
        pEncFbk[LEFT_LOWER_ARM]->getEncoderSpeeds(enc);
        
        v(8) = enc[0]; v(9) = enc[1]; v(10) = enc[2]; v(11) = enc[3];

        pEncFbk[RIGHT_UPPER_ARM]->getEncoderSpeeds(enc);

        v(12) = enc[0]; v(13) = enc[1]; v(14) = enc[2]; v(15) = enc[3]; 
        
        pEncFbk[RIGHT_LOWER_ARM]->getEncoderSpeeds(enc);
        
        v(16) = enc[0]; v(17) = enc[1]; v(18) = enc[2]; v(19) = enc[3];

        pEncFbk[HEAD]->getEncoderSpeeds(enc);

        v(20) = enc[0]; v(21) = enc[1];
    }
};

#endif