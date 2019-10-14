#include "odometry/angle.h"
#include <math.h>
#include <cmath>


namespace Odometry{
  double negativeFriendlyMod(double a, double b) {
    double r = fmod(a,b);
    return r < 0 ? r + b : r;
  }

  Angle::Angle(double radians){
    this->radianMeasure = radians;
  }

  Angle Angle::FromRadians(double radians){
    return Angle(radians);
  }

  Angle Angle::FromDegrees(double degrees){
    return Angle(degrees * 0.01745329251);
  }

  Angle Angle::FromTurns(double turn){
    return Angle(turn * 6.28318530718);
  }

  double Angle::GetRadianMeasure(){
    return this->radianMeasure;
  }

  double Angle::GetDegreeMeasure(){
    return this->radianMeasure / 0.01745329251;
  }

  double Angle::GetTurnMeasure(){
    return this->radianMeasure / 6.28318530718;
  }

  //Angle(const double& radian);
  //Angle operator=(const double radian);
  Angle::operator double(){
    return this->GetRadianMeasure();
  }

  Angle Angle::operator+(Angle a){
    return Angle::FromRadians(this->radianMeasure + a.radianMeasure);
  }

  Angle Angle::operator-(){
    return Angle::FromRadians(-(this->radianMeasure));
  }

  Angle Angle::operator-(Angle other){
    return Angle::FromRadians(this->radianMeasure - other.radianMeasure);
  }

  bool Angle::operator==(Angle other){
    return this->radianMeasure == other.radianMeasure;
  }

  double Angle::SignedAngleBetween(Angle reference, Angle b){
    Angle rawAngleBetween = b - reference;
    double positiveAngleBetween = negativeFriendlyMod(rawAngleBetween, 6.28318530718); //gonna be in the interval [0, 2pi)
    if (positiveAngleBetween <= 3.14159265359){
      return positiveAngleBetween;
    } else {
      return -6.28318530718 + positiveAngleBetween;
    }
  }

  double Angle::UnsignedAngleBetween(Angle a, Angle b){
    return fabs(Angle::SignedAngleBetween(a, b));
  }

  double Angle::getSin(){
    return sin(this->radianMeasure);
  }

  double Angle::getCos(){
    return cos(this->radianMeasure);
  }

  double Angle::getTan(){
    return tan(this->radianMeasure);
  }
}
