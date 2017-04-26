#ifndef __ROBOT_DRIVER_H__
#define __ROBOT_DRIVER_H__

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <Geometry.h>
#include <Matrix.h>

using namespace icub::robot_model;

#define MAX_ROBOT_PARTS 16

class IRobotDriver
{
public:
	virtual ~IRobotDriver(){}

	virtual void getPos(Matrix& q) = 0;
	virtual void setPos(const Matrix& q) = 0;
	virtual void getVel(Matrix& v) = 0;
	virtual void setVel(const Matrix& v) = 0;

	virtual void setVelocityMode(){}
	virtual void setPositionMode(){}
	virtual void setDirectMode(){}

	virtual bool open() = 0;
	virtual void close() = 0;
};

#if 0
class R1DriverFake : public IRobotDriver
{
public:
	R1DriverFake(int tau = 1)
	{
		TAU = tau;
		qout = new Matrix[TAU];
		qdotin = new Matrix[TAU];

		index_in = index_out = -1;

		qstate.resize(22);

		for (int i = 0; i < TAU; ++i)
		{
			qout[i].resize(22);
			qdotin[i].resize(22);
		}
	}

	virtual ~R1DriverFake()
	{
		delete[] qout;
		delete[] qdotin;
	}

	void getVel(Matrix &v)
	{
		v = qdotin[index_in];
	}

	void setVel(const Matrix &v)
	{
		qdotin[index_in] = v;
		index_in = (index_in + 1) % TAU;
		qstate += PERIOD*qdotin[index_in];
	}

	void getPos(Matrix &q)
	{
		qout[index_out] = qstate;
		index_out = (index_out + 1) % TAU;
		q = qout[index_out];
	}

	void setPos(const Matrix &q)
	{
		for (int i = 0; i < TAU; ++i) qout[i] = q;
		qstate = q;
		index_in = index_out = 0;
	}

	virtual bool open(){ return true; }
	virtual void close()
	{
		delete[] qout;
		delete[] qdotin;
	}

protected:
	Matrix *qout;
	Matrix *qdotin;

	Matrix qstate;

	int TAU;
	int index_in;
	int index_out;
};
#endif

class RobotDriver : public IRobotDriver
{
public:
	RobotDriver(std::string robotName, int nparts, const char** partnames) : mRobotName(robotName)
	{
		NPARTS = nparts;
		mPartName = partnames;

		for (int part = 0; part < NPARTS; ++part)
		{
			pEncFbk[part] = NULL;

			pPosCtrl[part] = NULL;
			pVelCtrl[part] = NULL;
			pDirCtrl[part] = NULL;

			pCmdCtrlMode[part] = NULL;

			mNumJoints[part] = 0;
		}
	}

	virtual ~RobotDriver(){ close(); }

	void modePosition(int part)
	{
		for (int j = 0; j<mNumJoints[part]; ++j) pCmdCtrlMode[part]->setPositionMode(j);
	}

	void modeDirect(int part)
	{
		for (int j = 0; j < mNumJoints[part]; ++j) pCmdCtrlMode[part]->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
	}

	void modeVelocity(int part)
	{
		for (int j = 0; j<mNumJoints[part]; ++j) pCmdCtrlMode[part]->setVelocityMode(j);
	}

	virtual void setVelocityMode()
	{
		for (int p = 0; p < NPARTS; ++p) modeVelocity(p);
	}

	virtual void setPositionMode()
	{
		for (int p = 0; p < NPARTS; ++p) modePosition(p);
	}

	virtual void setDirectMode()
	{
		for (int p = 0; p < NPARTS; ++p) modeDirect(p);
	}

	virtual bool open()
	{
		for (int part = 0; part<NPARTS; ++part)
		{
			mNumJoints[part] = 0;	

			mDriver[part] = openDriver(mPartName[part]);

			if (mDriver[part])
			{
				mDriver[part]->view(pEncFbk[part]);

				mDriver[part]->view(pPosCtrl[part]);
				mDriver[part]->view(pVelCtrl[part]);
				mDriver[part]->view(pDirCtrl[part]);

				mDriver[part]->view(pCmdCtrlMode[part]);

				if (pEncFbk[part]) pEncFbk[part]->getAxes(&mNumJoints[part]);
			}
			else
			{
				return false;
			}
		}

		return true;
	}

	void close()
	{
		for (int part = 0; part<NPARTS; ++part)
		{
			if (pCmdCtrlMode[part]) modePosition(part);
		}

		for (int part = 0; part < NPARTS; ++part)
		{
			mNumJoints[part] = 0;

			mDriver[part]->close();

			delete mDriver[part];

			mDriver[part] = NULL;
		}
	}

protected:
	yarp::dev::PolyDriver* mDriver[MAX_ROBOT_PARTS];

	int mNumJoints[MAX_ROBOT_PARTS];

	yarp::dev::IEncoders         *pEncFbk[MAX_ROBOT_PARTS];

	yarp::dev::IPositionControl2  *pPosCtrl[MAX_ROBOT_PARTS];
	yarp::dev::IVelocityControl2  *pVelCtrl[MAX_ROBOT_PARTS];
	yarp::dev::IPositionDirect    *pDirCtrl[MAX_ROBOT_PARTS];

	yarp::dev::IControlMode2      *pCmdCtrlMode[MAX_ROBOT_PARTS];

	std::string mRobotName;

	int NPARTS;
	const char **mPartName;

	yarp::dev::PolyDriver* openDriver(std::string part)
	{
		yarp::dev::PolyDriver *pDriver = NULL;

		yarp::os::Property options;
		options.put("robot", mRobotName.c_str());
		options.put("device", "remote_controlboard");
		options.put("local", (std::string("/driver/") + part).c_str());
		options.put("remote", (mRobotName + "/" + part).c_str());

		pDriver = new yarp::dev::PolyDriver(options);

		if (!pDriver) return NULL;

		if (!pDriver->isValid())
		{
			pDriver->close();
			delete pDriver;
			pDriver = NULL;
		}

		return pDriver;
	}
};

class R1DriverReal : public RobotDriver
{
public:
	enum R1Part { TORSO, TORSO_TRIPOD, HEAD, LEFT_ARM, LEFT_TRIPOD, RIGHT_ARM, RIGHT_TRIPOD, LEFT_HAND, RIGHT_HAND, NUM_R1_PARTS };

	R1DriverReal(std::string robotName) : RobotDriver(robotName, NUM_R1_PARTS, R1PartName){}

	virtual ~R1DriverReal(){}

	static const char *R1PartName[NUM_R1_PARTS];

	virtual void getPos(Matrix& q)
	{
		double enc[8];

		pEncFbk[TORSO_TRIPOD]->getEncoders(enc);

		q(0) = enc[0]; q(1) = enc[1]; q(2) = enc[2];

		pEncFbk[TORSO]->getEncoders(enc);

		q(3) = enc[3];

		pEncFbk[LEFT_ARM]->getEncoders(enc);

		q(4) = enc[0]; q(5) = enc[1]; q(6) = enc[2]; q(7) = enc[3]; q(8) = enc[4];

		pEncFbk[LEFT_TRIPOD]->getEncoders(enc);

		q(9) = enc[0]; q(10) = enc[1]; q(11) = enc[2];

		pEncFbk[RIGHT_ARM]->getEncoders(enc);

		q(12) = enc[0]; q(13) = enc[1]; q(14) = enc[2]; q(15) = enc[3]; q(16) = enc[4];

		pEncFbk[RIGHT_TRIPOD]->getEncoders(enc);

		q(17) = enc[0]; q(18) = enc[1]; q(19) = enc[2];

		pEncFbk[HEAD]->getEncoders(enc);

		q(20) = enc[0]; q(21) = enc[1];
	}

	virtual void getVel(Matrix& v)
	{
		double enc[8];

		pEncFbk[TORSO_TRIPOD]->getEncoderSpeeds(enc);

		v(0) = enc[0]; v(1) = enc[1]; v(2) = enc[2];

		pEncFbk[TORSO]->getEncoderSpeeds(enc);

		v(3) = enc[3];

		pEncFbk[LEFT_ARM]->getEncoderSpeeds(enc);

		v(4) = enc[0]; v(5) = enc[1]; v(6) = enc[2]; v(7) = enc[3]; v(8) = enc[4];

		pEncFbk[LEFT_TRIPOD]->getEncoderSpeeds(enc);

		v(9) = enc[0]; v(10) = enc[1]; v(11) = enc[2];

		pEncFbk[RIGHT_ARM]->getEncoderSpeeds(enc);

		v(12) = enc[0]; v(13) = enc[1]; v(14) = enc[2]; v(15) = enc[3]; v(16) = enc[4];

		pEncFbk[RIGHT_TRIPOD]->getEncoderSpeeds(enc);

		v(17) = enc[0]; v(18) = enc[1]; v(19) = enc[2];

		pEncFbk[HEAD]->getEncoderSpeeds(enc);

		v(20) = enc[0]; v(21) = enc[1];
	}

	virtual void setPos(const Matrix& q)
	{
		double pos[8];

		pos[0] = q(0); pos[1] = q(1); pos[2] = q(2);

		pPosCtrl[TORSO_TRIPOD]->positionMove(pos);

		pos[0] = q(3);

		static const int torso_yaw = 3;

		pPosCtrl[TORSO]->positionMove(1, &torso_yaw, pos);

		static const int left_upper[] = { 0, 1, 2, 3, 4 };

		pos[0] = q(4); pos[1] = q(5); pos[2] = q(6); pos[3] = q(7); pos[4] = q(8);

		pPosCtrl[LEFT_ARM]->positionMove(5, left_upper, pos);

		pos[0] = q(9); pos[1] = q(10); pos[2] = q(11);

		pPosCtrl[LEFT_TRIPOD]->positionMove(pos);

		static const int right_upper[] = { 0, 1, 2, 3, 4 };

		pos[0] = q(12); pos[1] = q(13); pos[2] = q(14); pos[3] = q(15); pos[4] = q(16);

		pPosCtrl[RIGHT_ARM]->positionMove(5, right_upper, pos);

		pos[0] = q(17); pos[1] = q(18); pos[2] = q(19);

		pPosCtrl[RIGHT_TRIPOD]->positionMove(pos);

		pos[0] = q(20); pos[1] = q(21);

		pPosCtrl[HEAD]->positionMove(pos);
	}

	virtual void setVel(const Matrix& v)
	{
		double vel[8];

		vel[0] = v(0); vel[1] = v(1); vel[2] = v(2);

		pVelCtrl[TORSO_TRIPOD]->velocityMove(vel);

		vel[0] = v(3);

		static const int torso_yaw = 3;

		pVelCtrl[TORSO]->velocityMove(1, &torso_yaw, vel);

		static const int left_upper[] = { 0, 1, 2, 3, 4 };

		vel[0] = v(4); vel[1] = v(5); vel[2] = v(6); vel[3] = v(7); vel[4] = v(8);

		pVelCtrl[LEFT_ARM]->velocityMove(5, left_upper, vel);

		vel[0] = v(9); vel[1] = v(10); vel[2] = v(11);

		pVelCtrl[LEFT_TRIPOD]->velocityMove(vel);

		static const int right_upper[] = { 0, 1, 2, 3, 4 };

		vel[0] = v(12); vel[1] = v(13); vel[2] = v(14); vel[3] = v(15); vel[4] = v(16);

		pVelCtrl[RIGHT_ARM]->velocityMove(5, right_upper, vel);

		vel[0] = v(17); vel[1] = v(18); vel[2] = v(19);

		pVelCtrl[RIGHT_TRIPOD]->velocityMove(vel);

		vel[0] = v(20); vel[1] = v(21);

		pVelCtrl[HEAD]->velocityMove(vel);
	}

	virtual bool open()
	{
		if (!RobotDriver::open()) return false;

		double ref_vel[MAX_ROBOT_PARTS][8];
		double ref_acc[MAX_ROBOT_PARTS][8];

		for (int part = 0; part<MAX_ROBOT_PARTS; ++part)
		{
			for (int j = 0; j<8; ++j)
			{
				ref_vel[part][j] = 15.0;
				ref_acc[part][j] = 100.0;
			}
		}

		ref_vel[TORSO][0] = 0.01;
		ref_acc[TORSO][0] = 0.05;

		ref_vel[TORSO_TRIPOD][0] = ref_vel[TORSO_TRIPOD][1] = ref_vel[TORSO_TRIPOD][2] = 0.01;
		ref_acc[TORSO_TRIPOD][0] = ref_acc[TORSO_TRIPOD][1] = ref_acc[TORSO_TRIPOD][2] = 0.05;

		ref_vel[LEFT_TRIPOD][0] = ref_vel[LEFT_TRIPOD][1] = ref_vel[LEFT_TRIPOD][2] = 0.005;
		ref_acc[LEFT_TRIPOD][0] = ref_acc[LEFT_TRIPOD][1] = ref_acc[LEFT_TRIPOD][2] = 0.025;

		ref_vel[RIGHT_TRIPOD][0] = ref_vel[RIGHT_TRIPOD][1] = ref_vel[RIGHT_TRIPOD][2] = 0.005;
		ref_acc[RIGHT_TRIPOD][0] = ref_acc[RIGHT_TRIPOD][1] = ref_acc[RIGHT_TRIPOD][2] = 0.025;

		for (int part = 0; part<NPARTS; ++part)
		{
			pPosCtrl[part]->setRefSpeeds(ref_vel[part]);
			pPosCtrl[part]->setRefAccelerations(ref_acc[part]);
			pVelCtrl[part]->setRefAccelerations(ref_acc[part]);

			modePosition(part);
		}

		return true;
	}

protected:

};

class R1DriverSim : public RobotDriver
{
public:
	enum R1Part { TORSO, HEAD, LEFT_ARM, LEFT_TRIPOD, RIGHT_ARM, RIGHT_TRIPOD, LEFT_HAND, RIGHT_HAND, NUM_R1_PARTS };

	R1DriverSim(std::string robotName, 
		const Matrix& tjeq, const Matrix& ljeq, const Matrix& rjeq) :
		RobotDriver(robotName, NUM_R1_PARTS, R1PartName), 
		TJeq(tjeq), LJeq(ljeq), RJeq(rjeq)
	{
	}

	static const char *R1PartName[NUM_R1_PARTS];

	virtual void getPos(Matrix &q)
	{
		//Matrix qin(22);

		double enc[8];

		pEncFbk[TORSO]->getEncoders(enc);

		q1q2q3T(0.09, enc[0], -enc[1], -enc[2], q(0), q(1), q(2));

		q(3) = enc[3];

		//qin(0) = enc[0]; qin(1) = enc[1]; qin(2) = enc[2]; qin(3) = enc[3];

		pEncFbk[LEFT_ARM]->getEncoders(enc);

		q(4) = enc[0]; q(5) = enc[1]; q(6) = enc[2]; q(7) = enc[3]; q(8) = enc[4];

		//qin(4) = enc[0]; qin(5) = enc[1]; qin(6) = enc[2]; qin(7) = enc[3]; qin(8) = enc[4];

		pEncFbk[LEFT_TRIPOD]->getEncoders(enc);

		//qin(9) = enc[0]; qin(10) = enc[1]; qin(11) = enc[2];

		q1q2q3W(0.018, enc[0], -enc[1], enc[2], q(9), q(10), q(11));

		pEncFbk[RIGHT_ARM]->getEncoders(enc);

		q(12) = enc[0]; q(13) = enc[1]; q(14) = enc[2]; q(15) = enc[3]; q(16) = enc[4];

		//qin(12) = enc[0]; qin(13) = enc[1]; qin(14) = enc[2]; qin(15) = enc[3]; qin(16) = enc[4];

		pEncFbk[RIGHT_TRIPOD]->getEncoders(enc);

		//qin(17) = enc[0]; qin(18) = enc[1]; qin(19) = enc[2];

		q1q2q3W(0.018, enc[0], -enc[1], -enc[2], q(17), q(18), q(19));

		pEncFbk[HEAD]->getEncoders(enc);

		q(20) = enc[0]; q(21) = enc[1];

		//qin(20) = enc[0]; qin(21) = enc[1];

		//qin.t().dump();
	}

	virtual void setPos(const Matrix &q)
	{
		//Matrix qout(22);

		double pos[80];

		double heave, roll, pitch;

		hrpT(0.09, q(0), q(1), q(2), heave, roll, pitch);

		pos[0] = heave; pos[1] = -roll; pos[2] = -pitch; pos[3] = q(3);

		//qout(0) = pos[0]; qout(1) = pos[1]; qout(2) = pos[2]; qout(3) = pos[3];

		pDirCtrl[TORSO]->setPositions(pos);

		static const int left_upper[] = { 0, 1, 2, 3, 4 };

		pos[0] = q(4); pos[1] = q(5); pos[2] = q(6); pos[3] = q(7); pos[4] = q(8);

		//qout(4) = pos[0]; qout(5) = pos[1]; qout(6) = pos[2]; qout(7) = pos[3]; qout(8) = pos[4];

		pDirCtrl[LEFT_ARM]->setPositions(5, left_upper, pos);

		hrpW(0.018, q(9), q(10), q(11), heave, roll, pitch);

		pos[0] = heave; pos[1] = -roll; pos[2] = pitch;

		//qout(9) = pos[0]; qout(10) = pos[1]; qout(11) = pos[2];

		pDirCtrl[LEFT_TRIPOD]->setPositions(pos);

		static const int right_upper[] = { 0, 1, 2, 3, 4 };

		pos[0] = q(12); pos[1] = q(13); pos[2] = q(14); pos[3] = q(15); pos[4] = q(16);

		//qout(12) = pos[0]; qout(13) = pos[1]; qout(14) = pos[2]; qout(15) = pos[3]; qout(16) = pos[4];

		pDirCtrl[RIGHT_ARM]->setPositions(5, right_upper, pos);

		hrpW(0.018, q(17), q(18), q(19), heave, roll, pitch);

		pos[0] = heave; pos[1] = -roll; pos[2] = -pitch;

		//qout(17) = pos[0]; qout(18) = pos[1]; qout(19) = pos[2];

		pDirCtrl[RIGHT_TRIPOD]->setPositions(pos);

		pos[0] = q(20); pos[1] = q(21);

		//qout(20) = pos[0]; qout(21) = pos[1];

		pDirCtrl[HEAD]->setPositions(pos);

		//qout.t().dump();

		//printf("***************************\n");
	}

	virtual void getVel(Matrix& v){}
	virtual void setVel(const Matrix& v){}

	virtual bool open()
	{
		if (!RobotDriver::open()) return false;

		double ref_vel[MAX_ROBOT_PARTS][8];
		double ref_acc[MAX_ROBOT_PARTS][8];

		for (int part = 0; part<MAX_ROBOT_PARTS; ++part)
		{
			for (int j = 0; j<8; ++j)
			{
				ref_vel[part][j] = 15.0;
				ref_acc[part][j] = 100.0;
			}
		}

		ref_vel[TORSO][0] = 0.01;
		ref_acc[TORSO][0] = 0.05;

		ref_vel[LEFT_TRIPOD][0] = 0.005;
		ref_acc[LEFT_TRIPOD][0] = 0.025;

		ref_vel[RIGHT_TRIPOD][0] = 0.005;
		ref_acc[RIGHT_TRIPOD][0] = 0.025;

		for (int part = 0; part<NPARTS; ++part)
		{
			pPosCtrl[part]->setRefSpeeds(ref_vel[part]);
			pPosCtrl[part]->setRefAccelerations(ref_acc[part]);
			pVelCtrl[part]->setRefAccelerations(ref_acc[part]);

			modePosition(part);
		}

		return true;
	}

protected:

	const Matrix& TJeq;
	const Matrix& LJeq;
	const Matrix& RJeq;

	void hrpT(double L, double q1, double q2, double q3, double &heave, double &roll, double &pitch)
	{
		double v1, v2, v3;

		hrp(L, q1, q2, q3, heave, v1, v2, v3);

		roll = RAD2DEG * atan2(-v2, v3);
		pitch = RAD2DEG * asin(v1);
	}

	void hrpW(double L, double q1, double q2, double q3, double &heave, double &roll, double &pitch)
	{
		double v1, v2, v3;

		hrp(L, q1, q2, q3, heave, v1, v2, v3);

		roll = RAD2DEG * atan2(v1, v3);
		pitch = RAD2DEG * asin(-v2);
	}

	void hrp(double L, double q1, double q2, double q3, double &heave, double &v1, double &v2, double &v3)
	{
		static const double SQRT3(sqrt(3.0));

		Vec3 N(q2 + q3 - 2.0*q1, SQRT3*(q3 - q2), 3.0*L);

		double N2 = N*N;
		double n = sqrt(N2);
		double k0 = 1.0 / n;

		double cosT = k0*N.z;
		double sinT = sqrt(1.0 - cosT*cosT);

		double Ux = 1.0, Uy = 0.0;

		if (sinT != 0.0)
		{
			Ux = -N.y / (n*sinT);
			Uy = N.x / (n*sinT);
		}

		double lcosT = 1.0 - cosT;
		double LcosT = L / cosT;
		double UxUx = Ux*Ux;
		double UxUy = Ux*Uy;
		double UyUy = Uy*Uy;

		Rotation R;

		R(0, 0) = UxUx*lcosT + cosT; R(0, 1) = UxUy*lcosT;        R(0, 2) = Uy*sinT;
		R(1, 0) = R(0, 1);           R(1, 1) = UyUy*lcosT + cosT; R(1, 2) = -Ux*sinT;
		R(2, 0) = -R(0, 2);          R(2, 1) = -R(1, 2);          R(2, 2) = cosT;

		heave = q1 - LcosT*(-0.5*R(0, 0) + 1.5*R(1, 1))*R(2, 0);

		v1 = R(0, 2);
		v2 = R(1, 2);
		v3 = R(2, 2);
	}

	void q1q2q3T(double L, double heave, double roll, double pitch, double &q1, double &q2, double &q3)
	{
		double cp = cos(DEG2RAD*pitch);
		double v1 = sin(DEG2RAD*pitch);
		double v2 = -cp*sin(DEG2RAD*roll);
		double v3 = cp*cos(DEG2RAD*roll);

		q1q3q3(L, heave, v1, v2, v3, q1, q2, q3);
	}

	void q1q2q3W(double L, double heave, double roll, double pitch, double &q1, double &q2, double &q3)
	{
		double cp = cos(DEG2RAD*pitch);
		double v1 = cp*sin(DEG2RAD*roll);
		double v2 = -sin(DEG2RAD*pitch);
		double v3 = cp*cos(DEG2RAD*roll);

		q1q3q3(L, heave, v1, v2, v3, q1, q2, q3);
	}

	void q1q3q3(double L, double heave, double v1, double v2, double v3, double &q1, double &q2, double &q3)
	{
		static const double SQRT3(sqrt(3.0));

		double ld2 = (v1*v2) / (1.0 + v3);

		ld2 = 1.0 - ld2*ld2;

		double LcosT = L / v3;

		q1 = heave - v1*LcosT*(1.5*sqrt(ld2 - v2*v2) - 0.5*sqrt(ld2 - v1*v1));

		double A = 3.0*v1*LcosT + 2.0*q1;
		double B = SQRT3*v2*LcosT;

		q3 = 0.5*(A + B);
		q2 = 0.5*(A - B);
	}
};

class iCubDriver : public RobotDriver
{
public:
	enum iCubPart { TORSO, HEAD, LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, NUM_ICUB_PARTS };

	iCubDriver(std::string robotName) : RobotDriver(robotName, NUM_ICUB_PARTS, iCubPartName){}

	virtual ~iCubDriver(){}

	static const char *iCubPartName[NUM_ICUB_PARTS];

	virtual void getPos(Matrix &q)
	{
		double enc[16];

		pEncFbk[TORSO]->getEncoders(enc);

		q(0) = enc[0]; q(1) = enc[1]; q(2) = enc[2];

		pEncFbk[LEFT_ARM]->getEncoders(enc);

		q(3) = enc[0]; q(4) = enc[1]; q(5) = enc[2]; q(6) = enc[3]; q(7) = enc[4]; q(8) = enc[5]; q(9) = enc[6];

		pEncFbk[RIGHT_ARM]->getEncoders(enc);

		q(10) = enc[0]; q(11) = enc[1]; q(12) = enc[2]; q(13) = enc[3]; q(14) = enc[4]; q(15) = enc[5]; q(16) = enc[6];

		pEncFbk[LEFT_LEG]->getEncoders(enc);

		q(17) = enc[0]; q(18) = enc[1]; q(19) = enc[2]; q(20) = enc[3]; q(21) = enc[4]; q(22) = enc[5];

		pEncFbk[RIGHT_LEG]->getEncoders(enc);

		q(23) = enc[0]; q(24) = enc[1]; q(25) = enc[2]; q(26) = enc[3]; q(27) = enc[4]; q(28) = enc[5];

		pEncFbk[HEAD]->getEncoders(enc);

		q(29) = enc[0]; q(30) = enc[1]; q(31) = enc[2];
	}

	virtual void setPos(const Matrix &q)
	{
		double pos[16];

		pos[0] = q(0); pos[1] = q(1); pos[2] = q(2);

		pPosCtrl[TORSO]->positionMove(pos);

		pos[0] = q(3); pos[1] = q(4); pos[2] = q(5); pos[3] = q(6); pos[4] = q(7); pos[5] = q(8); pos[6] = q(9);

		pPosCtrl[LEFT_ARM]->positionMove(pos);

		pos[0] = q(10); pos[1] = q(11); pos[2] = q(12); pos[3] = q(13); pos[4] = q(14); pos[5] = q(15); pos[6] = q(16);

		pPosCtrl[RIGHT_ARM]->positionMove(pos);

		pos[0] = q(17); pos[1] = q(18); pos[2] = q(19); pos[3] = q(20); pos[4] = q(21); pos[5] = q(22);

		pPosCtrl[LEFT_LEG]->positionMove(pos);

		pos[0] = q(23); pos[1] = q(24); pos[2] = q(25); pos[3] = q(26); pos[4] = q(27); pos[5] = q(28);

		pPosCtrl[RIGHT_LEG]->positionMove(pos);

		pos[0] = q(29); pos[1] = q(30); pos[2] = q(31);

		pPosCtrl[HEAD]->positionMove(pos);
	}

	virtual void setVel(const Matrix &v)
	{
		double vel[16];

		vel[0] = v(0); vel[1] = v(1); vel[2] = v(2);

		pVelCtrl[TORSO]->velocityMove(vel);

		vel[0] = v(3); vel[1] = v(4); vel[2] = v(5); vel[3] = v(6); vel[4] = v(7); vel[5] = v(8); vel[6] = v(9);

		pVelCtrl[LEFT_ARM]->velocityMove(vel);

		vel[0] = v(10); vel[1] = v(11); vel[2] = v(12); vel[3] = v(13); vel[4] = v(14); vel[5] = v(15); vel[6] = v(16);

		pVelCtrl[RIGHT_ARM]->velocityMove(vel);

		vel[0] = v(17); vel[1] = v(18); vel[2] = v(19); vel[3] = v(20); vel[4] = v(21); vel[5] = v(22);

		pVelCtrl[LEFT_LEG]->velocityMove(vel);

		vel[0] = v(23); vel[1] = v(24); vel[2] = v(25); vel[3] = v(26); vel[4] = v(27); vel[5] = v(28);

		pVelCtrl[RIGHT_LEG]->velocityMove(vel);

		vel[0] = v(29); vel[1] = v(30); vel[2] = v(31);

		pVelCtrl[HEAD]->velocityMove(vel);
	}

	virtual bool open()
	{
		if (!RobotDriver::open()) return false;

		double ref_vel[MAX_ROBOT_PARTS][16];
		double ref_acc[MAX_ROBOT_PARTS][16];

		for (int part = 0; part<MAX_ROBOT_PARTS; ++part)
		{
			for (int j = 0; j<8; ++j)
			{
				ref_vel[part][j] = 15.0;
				ref_acc[part][j] = 100.0;
			}
		}

		for (int part = 0; part<NPARTS; ++part)
		{
			pPosCtrl[part]->setRefSpeeds(ref_vel[part]);
			pPosCtrl[part]->setRefAccelerations(ref_acc[part]);
			pVelCtrl[part]->setRefAccelerations(ref_acc[part]);

			modePosition(part);
		}

		return true;
	}

protected:

};

#endif