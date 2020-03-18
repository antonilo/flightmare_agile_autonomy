#include <trajectory/fig8_trajectory.h>

#include <cmath>

using namespace RPGQ;

// constructor
Trajectory::Trajectory(std::shared_ptr<RPGQ::Timer> timer)
{
    // general trajectory variables
    timer_ = ExtTimer(timer);
    motionType_ = Motion::Figure8;
    startTime_ = -1.0;
    endTime_ = -1.0;
    initialized_ = false;

    posStart_ = Eigen::Vector3d(0.0,0.0,0.0);
    yawStart_ = 0.0;

    // figure 8
    fig8R_ = 6.0;
    fig8Omega_ = 1.5;
    fig8Dist_ = 10.0;
    fig8SplineAngle_ = 120.0/180.0*M_PI;

    double x0 = fig8R_*cos(fig8SplineAngle_ / 2.0) - fig8Dist_ / 2.0;
	double x0dot = fig8R_*fig8Omega_*sin(fig8SplineAngle_ / 2.0);
	double x0ddot = -fig8R_*fig8Omega_*fig8Omega_*cos(fig8SplineAngle_ / 2.0);
	double y0 = -fig8R_*sin(fig8SplineAngle_ / 2.0);
	double y0dot = fig8R_*fig8Omega_*cos(fig8SplineAngle_ / 2.0);
	double y0ddot = fig8R_*fig8Omega_*fig8Omega_*sin(fig8SplineAngle_ / 2.0);

	double x1 = -fig8R_*cos(fig8SplineAngle_ / 2.0) + fig8Dist_ / 2.0;
	double x1dot = fig8R_*fig8Omega_*sin(fig8SplineAngle_ / 2.0);
	double x1ddot = fig8R_*fig8Omega_*fig8Omega_*cos(fig8SplineAngle_ / 2.0);
	double y1 = fig8R_*sin(fig8SplineAngle_ / 2.0);
	double y1dot = fig8R_*fig8Omega_*cos(fig8SplineAngle_ / 2.0);
	double y1ddot = -fig8R_*fig8Omega_*fig8Omega_*sin(fig8SplineAngle_ / 2.0);

    fig8T_ = std::max(fig8SplineAngle_/fig8Omega_, sqrt((x0-x1)*(x0 - x1) + (y0-y1)*(y0 - y1))/(fig8Omega_*fig8R_));
	fig8Period_ = 2.0*fig8T_ + 2.0*(2.0*M_PI - fig8SplineAngle_) / fig8Omega_;
	fig8Duration_ = 5.0 * fig8Period_;

	double T = fig8T_ / 2.0;
	double T2 = T*T;
	double T3 = T2*T;
	double T4 = T3*T;
	double T5 = T4*T;
	double T6 = T5*T;
	double T7 = T6*T;
	double det = 16.0*T5;
	double AInv[6][6];

	AInv[0][0] = -3.0;			AInv[0][1] = -3.0*T;	AInv[0][2] = -T2;		AInv[0][3] = 3.0;		AInv[0][4] = -3.0*T;	AInv[0][5] = T2;
    AInv[1][0] = 0.0;           AInv[1][1] = T2;	    AInv[1][2] = T3;		AInv[1][3] = 0.0;		AInv[1][4] = -T2 ;		AInv[1][5] = T3;
	AInv[2][0] = 10.0*T2;		AInv[2][1] = 10.0*T3;	AInv[2][2] = 2.0* T4;   AInv[2][3] = -10.0*T2;	AInv[2][4] = 10.0*T3;	AInv[2][5] = -2.0*T4;
	AInv[3][0] = 0.0;			AInv[3][1] = -6.0*T4;	AInv[3][2] = -2.0*T5;	AInv[3][3] = 0.0;		AInv[3][4] = 6.0*T4;	AInv[3][5] = -2.0*T5;
	AInv[4][0] = -15.0*T4;	    AInv[4][1] = -7.0*T5;	AInv[4][2] = -T6;		AInv[4][3] = 15.0*T4;   AInv[4][4] = -7.0*T5;	AInv[4][5] = T6;
	AInv[5][0] = 8.0*T5;		AInv[5][1] = 5.0*T6;	AInv[5][2] = T7;		AInv[5][3] = 8.0*T5;	AInv[5][4] = -5.0*T6;	AInv[5][5] = T7;

	double b21x[6] = { x0, x0dot, x0ddot, x1, x1dot, x1ddot };
	double b21y[6] = { y0, y0dot, y0ddot, y1, y1dot, y1ddot };
	double b12x[6] = { -x0, -x0dot, -x0ddot, -x1, -x1dot, -x1ddot };
	double b12y[6] = { y0, y0dot, y0ddot, y1, y1dot, y1ddot };

	for (uint8_t m = 0; m < 6; m++)
	{
		fig8Coeff12x_[m] = 0.0;
		fig8Coeff12y_[m] = 0.0;
        fig8Coeff21x_[m] = 0.0;
		fig8Coeff21y_[m] = 0.0;

		for (uint8_t n = 0; n < 6; n++)
		{
			fig8Coeff12x_[m] += AInv[m][n] * b12x[n];
			fig8Coeff12y_[m] += AInv[m][n] * b12y[n];
            fig8Coeff21x_[m] += AInv[m][n] * b21x[n];
			fig8Coeff21y_[m] += AInv[m][n] * b21y[n];
		}

		fig8Coeff12x_[m] = fig8Coeff12x_[m] / det;
		fig8Coeff12y_[m] = fig8Coeff12y_[m] / det;
        fig8Coeff21x_[m] = fig8Coeff21x_[m] / det;
		fig8Coeff21y_[m] = fig8Coeff21y_[m] / det;
	}

    // auxiliary variables
    eZ_.setZero();
    eZ_(2) = 1.0;
    
    g_.setZero();
    g_(2) = 9.81;
}


// get functions
bool Trajectory::GetIsDone(void)
{
    return timer_.ElapsedSeconds() > endTime_ ? true : false;
}

TrajectoryState Trajectory::GetCurrentState(void)
{
    // compute (relative) time
    double t = timer_.ElapsedSeconds() - startTime_;

    // evaluate current motion
    TrajectoryState state;
    if (motionType_ == Motion::Figure8)
    {
        Figure8Trajectory(t, state.pos, state.vel, state.acc,
                          state.yaw, state.quat, state.omega, state.psi);
    }

    // add starting position and orientation
    state.pos = state.pos + posStart_;
    state.yaw = state.yaw + yawStart_;

    return state;
}


// set functions
bool Trajectory::StartMotion(Motion in)
{
    if (GetIsDone() && initialized_)
    {
        motionType_ = in;
        startTime_ = timer_.ElapsedSeconds();
        initialized_ = false;

        if (motionType_ == Motion::Figure8)
        {
            endTime_ = startTime_ + fig8Duration_;
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool Trajectory::SetStartPose(Eigen::Vector3d posStart, double yawStart)
{
    if (GetIsDone())
    {
        posStart_ = posStart;
        yawStart_ = yawStart;

        initialized_ = true;

        return true;
    }
    else
    {
        return false;
    }
}


// figure 8
void Trajectory::Figure8Trajectory(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc,
                                   double &yaw, Eigen::Quaterniond &quat, Eigen::Vector3d &omega, Eigen::Vector3d &psi)
{
    // check if trajectory is done
    if (t < 0.0 || fig8Duration_ < t)
    {
        // don't do anything
        return;
    }

	// compute time
    double tOriginal(t);
    t = std::fmod(t, fig8Period_);

    // compute translational trajectory
	if (t < fig8T_ / 2.0)
	{
		// on spline (circle 2 -> circle 1)
		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t3*t;
		double t5 = t4*t;

		pos(0) = fig8Coeff21x_[0] * t5 + fig8Coeff21x_[1] * t4 + fig8Coeff21x_[2] * t3 + fig8Coeff21x_[3] * t2 + fig8Coeff21x_[4] * t + fig8Coeff21x_[5];
		vel(0) = 5.0*fig8Coeff21x_[0] * t4 + 4.0*fig8Coeff21x_[1] * t3 + 3.0*fig8Coeff21x_[2] * t2 + 2.0*fig8Coeff21x_[3] * t + fig8Coeff21x_[4];
		acc(0) = 20.0*fig8Coeff21x_[0] * t3 + 12.0*fig8Coeff21x_[1] * t2 + 6.0*fig8Coeff21x_[2] * t + 2.0*fig8Coeff21x_[3];

		pos(1) = fig8Coeff21y_[0] * t5 + fig8Coeff21y_[1] * t4 + fig8Coeff21y_[2] * t3 + fig8Coeff21y_[3] * t2 + fig8Coeff21y_[4] * t + fig8Coeff21y_[5];
		vel(1) = 5.0*fig8Coeff21y_[0] * t4 + 4.0*fig8Coeff21y_[1] * t3 + 3.0*fig8Coeff21y_[2] * t2 + 2.0*fig8Coeff21y_[3] * t + fig8Coeff21y_[4];
		acc(1) = 20.0*fig8Coeff21y_[0] * t3 + 12.0*fig8Coeff21y_[1] * t2 + 6.0*fig8Coeff21y_[2] * t + 2.0*fig8Coeff21y_[3];
		 
	}
	else if (t < fig8T_ / 2.0 + (2.0*M_PI - fig8SplineAngle_) / fig8Omega_)
	{
		// on circle 1
		double alpha = M_PI - fig8SplineAngle_ / 2.0 - (t - fig8T_ / 2.0) * fig8Omega_;

		pos(0) = fig8R_*cos(alpha) + fig8Dist_ / 2.0;
		vel(0) = fig8R_*fig8Omega_*sin(alpha);
		acc(0) = -fig8R_*fig8Omega_*fig8Omega_*cos(alpha);

		pos(1) = fig8R_*sin(alpha);
		vel(1) = -fig8R_*fig8Omega_*cos(alpha);
		acc(1) = -fig8R_*fig8Omega_*fig8Omega_*sin(alpha);

	}
	else if (t < 3.0*fig8T_ / 2.0 + (2.0*M_PI - fig8SplineAngle_) / fig8Omega_)
	{
		// on spline (circle 1 -> circle 2)
		t = t - fig8T_ - (2.0*M_PI - fig8SplineAngle_) / fig8Omega_;

		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t3*t;
		double t5 = t4*t;

		pos(0) = fig8Coeff12x_[0] * t5 + fig8Coeff12x_[1] * t4 + fig8Coeff12x_[2] * t3 + fig8Coeff12x_[3] * t2 + fig8Coeff12x_[4] * t + fig8Coeff12x_[5];
		vel(0) = 5.0*fig8Coeff12x_[0] * t4 + 4.0*fig8Coeff12x_[1] * t3 + 3.0*fig8Coeff12x_[2] * t2 + 2.0*fig8Coeff12x_[3] * t + fig8Coeff12x_[4];
		acc(0) = 20.0*fig8Coeff12x_[0] * t3 + 12.0*fig8Coeff12x_[1] * t2 + 6.0*fig8Coeff12x_[2] * t + 2.0*fig8Coeff12x_[3];

		pos(1) = fig8Coeff12y_[0] * t5 + fig8Coeff12y_[1] * t4 + fig8Coeff12y_[2] * t3 + fig8Coeff12y_[3] * t2 + fig8Coeff12y_[4] * t + fig8Coeff12y_[5];
		vel(1) = 5.0*fig8Coeff12y_[0] * t4 + 4.0*fig8Coeff12y_[1] * t3 + 3.0*fig8Coeff12y_[2] * t2 + 2.0*fig8Coeff12y_[3] * t + fig8Coeff12y_[4];
		acc(1) = 20.0*fig8Coeff12y_[0] * t3 + 12.0*fig8Coeff12y_[1] * t2 + 6.0*fig8Coeff12y_[2] * t + 2.0*fig8Coeff12y_[3];

	}
	else if (t < 3.0*fig8T_ / 2.0 + 2.0*(2.0*M_PI - fig8SplineAngle_) / fig8Omega_)
	{
		// on circle 2
		double alpha = 2.0*M_PI + fig8SplineAngle_ / 2.0 + (t - 3.0*fig8T_ / 2.0 - (2.0*M_PI - fig8SplineAngle_)/fig8Omega_)*  fig8Omega_;

		pos(0) = fig8R_*cos(alpha) - fig8Dist_ / 2.0;
		vel(0) = -fig8R_*fig8Omega_*sin(alpha);
		acc(0) = -fig8R_*fig8Omega_*fig8Omega_*cos(alpha);

		pos(1) = fig8R_*sin(alpha);
		vel(1) = fig8R_*fig8Omega_*cos(alpha);
		acc(1) = -fig8R_*fig8Omega_*fig8Omega_*sin(alpha);
	}
	else
	{
		// on spline (circle 2 -> circle 1)
		t = t - 2.0*fig8T_ - 2.0*(2.0*M_PI - fig8SplineAngle_) / fig8Omega_;

		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t3*t;
		double t5 = t4*t;

		pos(0) = fig8Coeff21x_[0] * t5 + fig8Coeff21x_[1] * t4 + fig8Coeff21x_[2] * t3 + fig8Coeff21x_[3] * t2 + fig8Coeff21x_[4] * t + fig8Coeff21x_[5];
		vel(0) = 5.0*fig8Coeff21x_[0] * t4 + 4.0*fig8Coeff21x_[1] * t3 + 3.0*fig8Coeff21x_[2] * t2 + 2.0*fig8Coeff21x_[3] * t + fig8Coeff21x_[4];
		acc(0) = 20.0*fig8Coeff21x_[0] * t3 + 12.0*fig8Coeff21x_[1] * t2 + 6.0*fig8Coeff21x_[2] * t + 2.0*fig8Coeff21x_[3];

		pos(1) = fig8Coeff21y_[0] * t5 + fig8Coeff21y_[1] * t4 + fig8Coeff21y_[2] * t3 + fig8Coeff21y_[3] * t2 + fig8Coeff21y_[4] * t + fig8Coeff21y_[5];
		vel(1) = 5.0*fig8Coeff21y_[0] * t4 + 4.0*fig8Coeff21y_[1] * t3 + 3.0*fig8Coeff21y_[2] * t2 + 2.0*fig8Coeff21y_[3] * t + fig8Coeff21y_[4];
		acc(1) = 20.0*fig8Coeff21y_[0] * t3 + 12.0*fig8Coeff21y_[1] * t2 + 6.0*fig8Coeff21y_[2] * t + 2.0*fig8Coeff21y_[3];
	}

    // compute rotational trajectory
    // yaw always have x-axis point in direction of flight
    yaw = std::atan2(vel(1),vel(0));

    // quaternion
    quat = Eigen::Quaterniond::FromTwoVectors(eZ_, acc + g_)*Eigen::Quaterniond(std::cos(0.5*yaw),0.0,0.0,std::sin(0.5*yaw));

    // angular velocity
    double dt = 0.01;
    Eigen::Quaterniond quatFuture = Figure8AttitudeHelper(tOriginal + dt);
    Eigen::Quaterniond quatErr = quat.inverse()*quatFuture;
    if (quatErr.w() < 0.0)
    {
        quatFuture = quatFuture.coeffs()*(-1.0);
    }
    Eigen::Matrix<double,4,3> Q;
    Q(0,0) = -quat.x(); Q(0,1) = -quat.y(); Q(0,2) = -quat.z();
    Q(1,0) = quat.w(); Q(1,1) = -quat.z(); Q(1,2) = quat.y();
    Q(2,0) = quat.z(); Q(2,1) = quat.w(); Q(2,2) = -quat.x();
    Q(3,0) = -quat.y(); Q(3,1) = quat.x(); Q(3,2) = quat.w();
    Eigen::Vector4d quatDiff(quatFuture.w() - quat.w(), quatFuture.x() - quat.x(), quatFuture.y() - quat.y(), quatFuture.z() - quat.z());
    omega = 2.0/dt*Q.transpose()*quatDiff;

    // angular acceleration
    // TODO
}

Eigen::Quaterniond Trajectory::Figure8AttitudeHelper(double t)
{
    Eigen::Vector3d vel, acc;
    vel.setZero();
    acc.setZero();

    // compute time
    t = std::fmod(t, fig8Period_);

    // compute translational trajectory
	if (t < fig8T_ / 2.0)
	{
		// on spline (circle 2 -> circle 1)
		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t3*t;

		vel(0) = 5.0*fig8Coeff21x_[0] * t4 + 4.0*fig8Coeff21x_[1] * t3 + 3.0*fig8Coeff21x_[2] * t2 + 2.0*fig8Coeff21x_[3] * t + fig8Coeff21x_[4];
		acc(0) = 20.0*fig8Coeff21x_[0] * t3 + 12.0*fig8Coeff21x_[1] * t2 + 6.0*fig8Coeff21x_[2] * t + 2.0*fig8Coeff21x_[3];

		vel(1) = 5.0*fig8Coeff21y_[0] * t4 + 4.0*fig8Coeff21y_[1] * t3 + 3.0*fig8Coeff21y_[2] * t2 + 2.0*fig8Coeff21y_[3] * t + fig8Coeff21y_[4];
		acc(1) = 20.0*fig8Coeff21y_[0] * t3 + 12.0*fig8Coeff21y_[1] * t2 + 6.0*fig8Coeff21y_[2] * t + 2.0*fig8Coeff21y_[3];
		 
	}
	else if (t < fig8T_ / 2.0 + (2.0*M_PI - fig8SplineAngle_) / fig8Omega_)
	{
		// on circle 1
		double alpha = M_PI - fig8SplineAngle_ / 2.0 - (t - fig8T_ / 2.0) * fig8Omega_;

		vel(0) = fig8R_*fig8Omega_*sin(alpha);
		acc(0) = -fig8R_*fig8Omega_*fig8Omega_*cos(alpha);

		vel(1) = -fig8R_*fig8Omega_*cos(alpha);
		acc(1) = -fig8R_*fig8Omega_*fig8Omega_*sin(alpha);

	}
	else if (t < 3.0*fig8T_ / 2.0 + (2.0*M_PI - fig8SplineAngle_) / fig8Omega_)
	{
		// on spline (circle 1 -> circle 2)
		t = t - fig8T_ - (2.0*M_PI - fig8SplineAngle_) / fig8Omega_;

		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t3*t;

		vel(0) = 5.0*fig8Coeff12x_[0] * t4 + 4.0*fig8Coeff12x_[1] * t3 + 3.0*fig8Coeff12x_[2] * t2 + 2.0*fig8Coeff12x_[3] * t + fig8Coeff12x_[4];
		acc(0) = 20.0*fig8Coeff12x_[0] * t3 + 12.0*fig8Coeff12x_[1] * t2 + 6.0*fig8Coeff12x_[2] * t + 2.0*fig8Coeff12x_[3];

		vel(1) = 5.0*fig8Coeff12y_[0] * t4 + 4.0*fig8Coeff12y_[1] * t3 + 3.0*fig8Coeff12y_[2] * t2 + 2.0*fig8Coeff12y_[3] * t + fig8Coeff12y_[4];
		acc(1) = 20.0*fig8Coeff12y_[0] * t3 + 12.0*fig8Coeff12y_[1] * t2 + 6.0*fig8Coeff12y_[2] * t + 2.0*fig8Coeff12y_[3];

	}
	else if (t < 3.0*fig8T_ / 2.0 + 2.0*(2.0*M_PI - fig8SplineAngle_) / fig8Omega_)
	{
		// on circle 2
		double alpha = 2.0*M_PI + fig8SplineAngle_ / 2.0 + (t - 3.0*fig8T_ / 2.0 - (2.0*M_PI - fig8SplineAngle_)/fig8Omega_)*  fig8Omega_;

		vel(0) = -fig8R_*fig8Omega_*sin(alpha);
		acc(0) = -fig8R_*fig8Omega_*fig8Omega_*cos(alpha);

		vel(1) = fig8R_*fig8Omega_*cos(alpha);
		acc(1) = -fig8R_*fig8Omega_*fig8Omega_*sin(alpha);
	}
	else
	{
		// on spline (circle 2 -> circle 1)
		t = t - 2.0*fig8T_ - 2.0*(2.0*M_PI - fig8SplineAngle_) / fig8Omega_;

		double t2 = t*t;
		double t3 = t2*t;
		double t4 = t3*t;

		vel(0) = 5.0*fig8Coeff21x_[0] * t4 + 4.0*fig8Coeff21x_[1] * t3 + 3.0*fig8Coeff21x_[2] * t2 + 2.0*fig8Coeff21x_[3] * t + fig8Coeff21x_[4];
		acc(0) = 20.0*fig8Coeff21x_[0] * t3 + 12.0*fig8Coeff21x_[1] * t2 + 6.0*fig8Coeff21x_[2] * t + 2.0*fig8Coeff21x_[3];

		vel(1) = 5.0*fig8Coeff21y_[0] * t4 + 4.0*fig8Coeff21y_[1] * t3 + 3.0*fig8Coeff21y_[2] * t2 + 2.0*fig8Coeff21y_[3] * t + fig8Coeff21y_[4];
		acc(1) = 20.0*fig8Coeff21y_[0] * t3 + 12.0*fig8Coeff21y_[1] * t2 + 6.0*fig8Coeff21y_[2] * t + 2.0*fig8Coeff21y_[3];
	}

    // compute rotational trajectory
    // yaw always have x-axis point in direction of flight
    double yaw = std::atan2(vel(1),vel(0));

    // quaternion
    return Eigen::Quaterniond::FromTwoVectors(eZ_, acc + g_)*Eigen::Quaterniond(std::cos(0.5*yaw),0.0,0.0,std::sin(0.5*yaw));
}