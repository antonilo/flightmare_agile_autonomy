// rpgq common
#include <rpgq_common/time/ext_timer.h>

// others
#include <eigen3/Eigen/Dense>

struct TrajectoryState
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;

    double yaw;
    Eigen::Quaterniond quat;
    Eigen::Vector3d omega;
    Eigen::Vector3d psi;

    // constructor
    TrajectoryState(void)
    {
        pos.setZero();
        vel.setZero();
        acc.setZero();

        yaw = 0.0;
        quat.setIdentity();
        omega.setZero();
        psi.setZero();
    }
};


class Trajectory
{
public:
    // constructor & destructor
    Trajectory(std::shared_ptr<RPGQ::Timer> timer);
    ~Trajectory(void) {};

    // public types
    enum Motion
    {
        Figure8 = 0
    };

    // public get functions
    bool GetIsDone(void);
    TrajectoryState GetCurrentState(void);

    // public set functions
    bool StartMotion(Motion in);
    bool SetStartPose(Eigen::Vector3d posStart, double yawStart);

private:
    // general trajectory variables
    RPGQ::ExtTimer timer_;
    Motion motionType_;
    double startTime_;
    double endTime_;
    bool initialized_;

    Eigen::Vector3d posStart_;
    double yawStart_;

    // figure 8
    double fig8Duration_;
    double fig8Period_;
    double fig8R_;
    double fig8Omega_;
    double fig8Dist_;
    double fig8SplineAngle_;
    double fig8T_;
    double fig8Coeff21x_[6], fig8Coeff21y_[6];
	double fig8Coeff12x_[6], fig8Coeff12y_[6];

    void Figure8Trajectory(double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc,
                           double &yaw, Eigen::Quaterniond &quat, Eigen::Vector3d &omega, Eigen::Vector3d &psi);
    Eigen::Quaterniond Figure8AttitudeHelper(double t);

    // auxiliary variables
    Eigen::Vector3d eZ_, g_;
};