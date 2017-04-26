/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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

#include <Joints.h>

using namespace icub::robot_model;

void Link::setPose(Matrix& q, const Transform& Tprec, Component *from)
{
	Tparent = Tprec;

	if (from == mParent)
	{
		Toj = Tprec*Tdir;

		for (int i = 0; i < Nchilds; ++i) mChilds[i]->setPose(q, Toj, this);
	}
	else
	{
		Toj = Tprec;

		Transform T = Tprec*Tinv;

		if (mParent) mParent->setPose(q, T, this);

		for (int i = 0; i < Nchilds; ++i)
		{
			if (from != mChilds[i]) mChilds[i]->setPose(q, T, this);
		}
	}
}

void Link::setPoseCalcJ(Matrix& q, const Transform& Tprec, Component *from, Vec3* Poj_1, Vec3* Zoj_1, Vec3* Voj_1)
{
	Tparent = Tprec;

	if (!Poj) Poj = new Vec3[mRoot->NJ];
	if (!Zoj) Zoj = new Vec3[mRoot->NJ];
	if (!Voj) Voj = new Vec3[mRoot->NJ];

	if (Zoj_1 && Poj_1 && Voj_1)
	{
		JOINTS(j) { Poj[j] = Poj_1[j]; Zoj[j] = Zoj_1[j]; Voj[j] = Voj_1[j]; }
	}
	else
	{
		JOINTS(j) { Poj[j].clear(); Zoj[j].clear(); Voj[j].clear(); }
	}

	if (from == mParent)
	{
		Toj = Tprec*Tdir;

		for (int i = 0; i < Nchilds; ++i) mChilds[i]->setPoseCalcJ(q, Toj, this, Poj, Zoj, Voj);
	}
	else
	{
		Toj = Tprec;

		Transform T = Tprec*Tinv;

		if (mParent) mParent->setPoseCalcJ(q, T, this, Poj, Zoj, Voj);

		for (int i = 0; i < Nchilds; ++i)
		{
			if (from != mChilds[i]) mChilds[i]->setPoseCalcJ(q, T, this, Poj, Zoj, Voj);
		}
	}
}

void RotJoint::setPose(Matrix& q, const Transform& Tprec, Component *from)
{
	Tparent = Tprec;

	if (from == mParent)
	{
		Toj = Tprec*Transform(q(j0));

		Poj[j0] = Toj.Pj();
		Zoj[j0] = Toj.Zj();
		Voj[j0].clear();

		for (int i = 0; i < Nchilds; ++i) mChilds[i]->setPose(q, Toj, this);
	}
	else
	{
		Toj = Tprec*Transform(-q(j0));

		Poj[j0] = Toj.Pj();
		Zoj[j0] = -Toj.Zj();
		Voj[j0].clear();

		if (mParent) mParent->setPose(q, Toj, this);

		for (int i = 0; i < Nchilds; ++i)
		{
			if (from != mChilds[i]) mChilds[i]->setPose(q, Toj, this);
		}
	}
}

void RotJoint::setPoseCalcJ(Matrix& q, const Transform& Tprec, Component *from, Vec3* Poj_1, Vec3* Zoj_1, Vec3* Voj_1)
{
	Tparent = Tprec;

	if (!Poj) Poj = new Vec3[mRoot->NJ];
	if (!Zoj) Zoj = new Vec3[mRoot->NJ];
	if (!Voj) Voj = new Vec3[mRoot->NJ];

	if (Zoj_1 && Poj_1 && Voj_1)
	{
		JOINTS(j) { Poj[j] = Poj_1[j]; Zoj[j] = Zoj_1[j]; Voj[j] = Voj_1[j]; }
	}
	else
	{
		JOINTS(j) { Poj[j].clear(); Zoj[j].clear(); Voj[j].clear(); }
	}

	if (from == mParent)
	{
		Toj = Tprec*Transform(q(j0));

		Poj[j0] = Toj.Pj();
		Zoj[j0] = Toj.Zj();
		Voj[j0].clear();

		for (int i = 0; i < Nchilds; ++i) mChilds[i]->setPoseCalcJ(q, Toj, this, Poj, Zoj, Voj);
	}
	else
	{
		Toj = Tprec*Transform(-q(j0));

		Poj[j0] = Toj.Pj();
		Zoj[j0] = -Toj.Zj();
		Voj[j0].clear();

		if (mParent) mParent->setPoseCalcJ(q, Toj, this, Poj, Zoj, Voj);

		for (int i = 0; i < Nchilds; ++i)
		{
			if (from != mChilds[i]) mChilds[i]->setPoseCalcJ(q, Toj, this, Poj, Zoj, Voj);
		}
	}
}

void Trifid::q1q2q3(double heave, double roll, double pitch, double &q1, double &q2, double &q3)
{
	static const double SQRT3(sqrt(3.0));

	double cp = cos(DEG2RAD*pitch);
	double v1 = sin(DEG2RAD*pitch);

	double v2 = -cp*sin(DEG2RAD*roll);
	double v3 =  cp*cos(DEG2RAD*roll);

	double ld2 = (v1*v2) / (1.0 + v3);

	ld2 = 1.0 - ld2*ld2;

	double LcosT = L / v3;

	q1 = heave - v1*LcosT*(1.5*sqrt(ld2 - v2*v2) - 0.5*sqrt(ld2 - v1*v1));

	double A = 3.0*v1*LcosT + 2.0*q1;
	double B = SQRT3*v2*LcosT;

	q3 = 0.5*(A+B);
	q2 = 0.5*(A-B);
}

void Trifid::setPose(Matrix& q,const Transform& Tprec,Component *from)
{
    Tparent = Tprec;

    static const double SQRT3(sqrt(3.0));

    Transform T0;

    {
		Vec3 N(q(j1) + q(j2) - 2.0*q(j0), SQRT3*(q(j2) - q(j1)), 3.0*L);

		double n=N.mod();
        double k0=1.0/n;

        double cosT=k0*N.z;
        double sinT=sqrt(1.0-cosT*cosT);

        double Ux=1.0,Uy=0.0;

        if (sinT!=0.0)
        {
            Ux=-N.y/(n*sinT);
            Uy= N.x/(n*sinT);
        }

        double lcosT=1.0-cosT;

        Rotation& R=T0.Rj();

        R(0,0)=Ux*Ux*lcosT+cosT; R(0,1)=Ux*Uy*lcosT;      R(0,2)= Uy*sinT;
        R(1,0)= R(0,1);         R(1,1)=Uy*Uy*lcosT+cosT; R(1,2)=-Ux*sinT;
        R(2,0)=-R(0,2);         R(2,1)=-R(1,2);         R(2,2)=    cosT;

		T0.Pj() = Vec3(L, 0.0, q(j0)) - (L/cosT)*(-0.5*R(0, 0) + 1.5*R(1, 1))*R.Ex();

        //double theta=atan2(sinT,cosT);
        //AA.x=theta*Ux;
        //AA.y=theta*Uy;
        //AA.z=0.0;
    }

    if (from==mParent)
    {
        Toj=Tprec*T0;

        for (int i=0; i<Nchilds; ++i) mChilds[i]->setPose(q,Toj,this);
    }
    else
    {
        Toj=Tprec*T0.inv();

        if (mParent) mParent->setPose(q,Toj,this);

        for (int i=0; i<Nchilds; ++i)
        {
            if (from!=mChilds[i]) mChilds[i]->setPose(q,Toj,this);
        }
    }
}

void Trifid::setPoseCalcJ(Matrix& q, const Transform& Tprec, Component *from, Vec3* Poj_1, Vec3* Zoj_1, Vec3* Voj_1)
{
	Tparent = Tprec;

	static const double SQRT3(sqrt(3.0));

	if (!Poj) Poj = new Vec3[mRoot->NJ];
	if (!Zoj) Zoj = new Vec3[mRoot->NJ];
	if (!Voj) Voj = new Vec3[mRoot->NJ];

	if (Zoj_1 && Poj_1)
	{
		JOINTS(j) { Poj[j] = Poj_1[j]; Zoj[j] = Zoj_1[j]; Voj[j] = Voj_1[j]; }
	}
	else
	{
		JOINTS(j) { Poj[j].clear(); Zoj[j].clear(); Voj[j].clear(); }
	}

	Transform T0;

	{
		double q1 = q(j0), q2 = q(j1), q3 = q(j2);

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

		Rotation& R = T0.Rj();

		R(0, 0) = UxUx*lcosT + cosT; R(0, 1) = UxUy*lcosT;      R(0, 2) = Uy*sinT;
		R(1, 0) = R(0, 1);         R(1, 1) = UyUy*lcosT + cosT; R(1, 2) = -Ux*sinT;
		R(2, 0) = -R(0, 2);         R(2, 1) = -R(1, 2);         R(2, 2) = cosT;

		double m0 = LcosT*(-0.5*R(0, 0) + 1.5*R(1, 1));

		Vec3& P = T0.Pj();
		P = Vec3(L, 0.0, q1) - m0*R.Ex();

		//double theta = atan2(sinT, cosT);
		//AA.x = theta*Ux;
		//AA.y = theta*Uy;
		//AA.z = 0.0;

		////////////////////////////////////////////////////////////////////////////

		double k1 = 2.0*k0;
		double dNdq1 = k1*(2.0*q1 - q2 - q3);
		double dNdq2 = k1*(2.0*q2 - q3 - q1);
		double dNdq3 = k1*(2.0*q3 - q1 - q2);

		double k2 = k0*k0;
		Matrix dRdq1(3, 3), dRdq2(3, 3), dRdq3(3, 3);

		dRdq1(0, 2) = (-2.0*n - N.x*dNdq1)*k2; // d(Nx/N)/dq1
		dRdq2(0, 2) = (n - N.x*dNdq2)*k2; // d(Nx/N)/dq2
		dRdq3(0, 2) = (n - N.x*dNdq3)*k2; // d(Nx/N)/dq3

		dRdq1(1, 2) = (-N.y*dNdq1)*k2; // d(Ny/N)/dq1
		dRdq2(1, 2) = (-SQRT3*n - N.y*dNdq2)*k2; // d(Ny/N)/dq2
		dRdq3(1, 2) = (SQRT3*n - N.y*dNdq3)*k2; // d(Ny/N)/dq3

		dRdq1(2, 2) = (-N.z*dNdq1)*k2; // dCosT/dq1
		dRdq2(2, 2) = (-N.z*dNdq2)*k2; // dCosT/dq2
		dRdq3(2, 2) = (-N.z*dNdq3)*k2; // dCosT/dq3

		dRdq1(2, 0) = -dRdq1(0, 2);
		dRdq2(2, 0) = -dRdq2(0, 2);
		dRdq3(2, 0) = -dRdq3(0, 2);

		dRdq1(2, 1) = -dRdq1(1, 2);
		dRdq2(2, 1) = -dRdq2(1, 2);
		dRdq3(2, 1) = -dRdq3(1, 2);

		////////////////////////////////////////

		double Nx2Ny2 = N.x*N.x + N.y*N.y;

		if (Nx2Ny2>0.0)
		{
			double k3 = 1.0 / (Nx2Ny2*Nx2Ny2);
			double k4 = 4.0*N.x*N.y;
			double dNxNydq1 = k3*(Nx2Ny2*(-2.0*N.y) - k4*(2.0*q1 - q2 - q3));
			double dNxNydq2 = k3*(Nx2Ny2*(-SQRT3*N.x + N.y) - k4*(2.0*q2 - q3 - q1));
			double dNxNydq3 = k3*(Nx2Ny2*(SQRT3*N.x + N.y) - k4*(2.0*q3 - q1 - q2));

			dRdq1(1, 0) = dRdq1(0, 1) = -lcosT*dNxNydq1 - UxUy*dRdq1(2, 2);
			dRdq2(1, 0) = dRdq2(0, 1) = -lcosT*dNxNydq2 - UxUy*dRdq2(2, 2);
			dRdq3(1, 0) = dRdq3(0, 1) = -lcosT*dNxNydq3 - UxUy*dRdq3(2, 2);

			{
				double k5 = k3*N.x*N.x*2.0*N.y*SQRT3;
				double k6 = k3*N.y*N.y*2.0*N.x;
				double dUx2dq1 = (2.0*k6)*lcosT;
				double dUx2dq2 = (-k5 - k6)*lcosT;
				double dUx2dq3 = (k5 - k6)*lcosT;

				dRdq1(0, 0) = dUx2dq1 + UyUy*dRdq1(2, 2);
				dRdq2(0, 0) = dUx2dq2 + UyUy*dRdq2(2, 2);
				dRdq3(0, 0) = dUx2dq3 + UyUy*dRdq3(2, 2);

				dRdq1(1, 1) = -dUx2dq1 + UxUx*dRdq1(2, 2);
				dRdq2(1, 1) = -dUx2dq2 + UxUx*dRdq2(2, 2);
				dRdq3(1, 1) = -dUx2dq3 + UxUx*dRdq3(2, 2);
			}

			Matrix Rt = R.inv();

			Matrix S1 = dRdq1*Rt;
			Matrix S2 = dRdq2*Rt;
			Matrix S3 = dRdq3*Rt;

			//S1=0.5*(S1-S1.t());
			//S2=0.5*(S2-S2.t());
			//S3=0.5*(S3-S3.t());

			Z[0] = Vec3(S1(2, 1), S1(0, 2), S1(1, 0));
			Z[1] = Vec3(S2(2, 1), S2(0, 2), S2(1, 0));
			Z[2] = Vec3(S3(2, 1), S3(0, 2), S3(1, 0));

			double k7 = (L / (cosT*cosT))*(-0.5*R(0, 0) + 1.5*R(1, 1));

			double dm0dq1 = LcosT*(-0.5*dRdq1(0, 0) + 1.5*dRdq1(1, 1)) - k7*dRdq1(2, 2);
			double dm0dq2 = LcosT*(-0.5*dRdq2(0, 0) + 1.5*dRdq2(1, 1)) - k7*dRdq2(2, 2);
			double dm0dq3 = LcosT*(-0.5*dRdq3(0, 0) + 1.5*dRdq3(1, 1)) - k7*dRdq3(2, 2);

			Vec3 Ex = R.Ex();

			static const Vec3 Ez(0.0, 0.0, 1.0);

			V[0] = Ez - dm0dq1*Ex - m0*Vec3(dRdq1(0, 0), dRdq1(1, 0), dRdq1(2, 0));
			V[1] = -dm0dq2*Ex - m0*Vec3(dRdq2(0, 0), dRdq2(1, 0), dRdq2(2, 0));
			V[2] = -dm0dq3*Ex - m0*Vec3(dRdq3(0, 0), dRdq3(1, 0), dRdq3(2, 0));
		}
		else
		{
			static const Vec3 V3(0.0, 0.0, 1.0 / 3.0);

			double k1 = -0.5*L;
			double k2 = 0.5*SQRT3*L;

			Vec3 P1(L, 0.0, q1);
			Vec3 P2(k1, k2, q2);
			Vec3 P3(k1, -k2, q3);

			double Kl = 2.0 / (3.0*L);
			Z[0] = (P3 - P2).norm(Kl);
			Z[1] = (P1 - P3).norm(Kl);
			Z[2] = (P2 - P1).norm(Kl);

			V[0] = V[1] = V[2] = V3;
		}

		//double roll = atan2(-R(1, 2), R(2, 2));
		//double pitch = atan2(R(0,2),R(2,2)*cos(roll)-R(1,2)*sin(roll));
		//double yaw = atan2(-R(0,1), R(0,0));

		//Vec3 Zp(-sin(roll), cos(roll), 0.0);

		//Jeq(0, 0) = V[0].z; Jeq(0, 1) = V[1].z; Jeq(0, 2) = V[2].z;
		//Jeq(1, 0) = Z[0].x; Jeq(1, 1) = Z[1].x; Jeq(1, 2) = Z[2].x;
		//Jeq(2, 0) = Zp*Z[0]; Jeq(2, 1) = Zp*Z[1]; Jeq(2, 2) = Zp*Z[2];

		//roll *= RAD2DEG;
		//pitch *= RAD2DEG;
		//yaw *= RAD2DEG;

		//HRPeq = Vec3(P.z, roll, pitch);

		//double l1, l2, l3;
		//q1q2q3(P.z, roll, pitch, l1, l2, l3);
		//printf("%.12lf %.12lf %.12lf\n", q1, q2, q3);
		//printf("%.12lf %.12lf %.12lf\n\n", l1, l2, l3);

		////////////////////////////////////////////////////
	}

	if (from == mParent)
	{
		Toj = Tprec*T0;

		V[0] = Tprec.Rj()*V[0];
		V[1] = Tprec.Rj()*V[1];
		V[2] = Tprec.Rj()*V[2];

		Poj[j0] = Toj.Pj();
		Zoj[j0] = Tprec.Rj()*Z[0];
		Voj[j0] = Tprec.Rj()*V[0];

		Poj[j1] = Toj.Pj();
		Zoj[j1] = Tprec.Rj()*Z[1];
		Voj[j1] = Tprec.Rj()*V[1];

		Poj[j2] = Toj.Pj();
		Zoj[j2] = Tprec.Rj()*Z[2];
		Voj[j2] = Tprec.Rj()*V[2];

		Voj[j0].clear();
		Voj[j1].clear();
		Voj[j2].clear();

		for (int i = 0; i<Nchilds; ++i) mChilds[i]->setPoseCalcJ(q, Toj, this, Poj, Zoj, Voj);
	}
	else
	{
		Toj = Tprec*T0.inv();

		V[0] = Tprec.Rj()*V[0];
		V[1] = Tprec.Rj()*V[1];
		V[2] = Tprec.Rj()*V[2];

		Poj[j0] = Toj.Pj();
		Zoj[j0] = Toj.Rj()*-Z[0];
		Voj[j0] = -V[0];

		Poj[j1] = Toj.Pj();
		Zoj[j1] = Toj.Rj()*-Z[1];
		Voj[j1] = -V[1];

		Poj[j2] = Toj.Pj();
		Zoj[j2] = Toj.Rj()*-Z[2];
		Voj[j2] = -V[2];

		Voj[j0].clear();
		Voj[j1].clear();
		Voj[j2].clear();

		if (mParent) mParent->setPoseCalcJ(q, Toj, this, Poj, Zoj, Voj);

		for (int i = 0; i<Nchilds; ++i)
		{
			if (from != mChilds[i]) mChilds[i]->setPoseCalcJ(q, Toj, this, Poj, Zoj, Voj);
		}
	}
}
