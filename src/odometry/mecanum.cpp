#include "odometry/mecanum.h"
#include <cmath>

#include "odometry/odometryMovement.h"

namespace Odometry::Mecanum {
  
  MecanumMotorSpeeds::MecanumMotorSpeeds(double frontLeft, double backLeft, double frontRight, double backRight){
    this->frontLeft = frontLeft;
    this->backLeft = backLeft;
    this->frontRight = frontRight;
    this->backRight = backRight;
  }

  MecanumMotorSpeeds MecanumMotorSpeeds::operator+(MecanumMotorSpeeds other){
    return MecanumMotorSpeeds(this->frontLeft + other.frontLeft, this->backLeft + other.backLeft, this->frontRight + other.frontRight, this->backRight + other.backRight);
  }

  MecanumMotorSpeeds MecanumMotorSpeeds::operator*(double other){
    return MecanumMotorSpeeds(this->frontLeft * other, this->backLeft * other, this->frontRight * other, this->backRight * other);
  }

  MecanumMotorSpeeds MecanumMotorSpeeds::normalize(double max){
    double m = max;
    if (fabs(this->frontLeft) > m){
      m = fabs(this->frontLeft);
    }
    if (fabs(this->backLeft) > m){
      m = fabs(this->backLeft);
    }
    if (fabs(this->frontRight) > m){
      m = fabs(this->frontRight);
    }
    if (fabs(this->backRight) > m){
      m = fabs(this->backRight);
    }

    double factor = max / m;

    return *this * factor;
  }

  void MecanumMotorSpeeds::ApplyToMotors(OdometryMotor& frontLeft, OdometryMotor& backLeft, OdometryMotor& frontRight, OdometryMotor& backRight){
    frontLeft.move_velocity(this->frontLeft);
    backLeft.move_velocity(this->backLeft);
    frontRight.move_velocity(this->frontRight);
    backRight.move_velocity(this->backRight);
  }
}
