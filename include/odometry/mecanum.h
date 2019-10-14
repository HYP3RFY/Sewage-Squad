#ifndef MECANUM_ODOMETRY_H
#define MECANUM_ODOMETRY_H

#include "odometry/odometryMovement.h"

using namespace Odometry::Movement;

namespace Odometry::Mecanum {

  struct MecanumMotorSpeeds {
  public:
    MecanumMotorSpeeds(double frontLeft, double backLeft, double frontRight, double backRight);

    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;

    MecanumMotorSpeeds operator+(MecanumMotorSpeeds other);
    MecanumMotorSpeeds operator*(double other);

    MecanumMotorSpeeds normalize(double max);

    void ApplyToMotors(OdometryMotor& frontLeft, OdometryMotor& backLeft, OdometryMotor& frontRight, OdometryMotor& backRight);
  };
}
#endif
