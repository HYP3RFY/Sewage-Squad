#ifndef ANGLE_H
#define ANGLE_H

namespace Odometry{
struct Angle{
private:
  double radianMeasure;
public:
  Angle(double radians);
  static Angle FromRadians(double radians);
  static Angle FromDegrees(double degrees);
  static Angle FromTurns(double turn);

  double GetRadianMeasure();
  double GetDegreeMeasure();
  double GetTurnMeasure();


  operator double();

  Angle operator+(Angle a);
  Angle operator-();
  Angle operator-(Angle other);
  bool operator==(Angle other);

  static double SignedAngleBetween(Angle reference, Angle b);
  static double UnsignedAngleBetween(Angle a, Angle b);

  double getSin();
  double getCos();
  double getTan();
};
}
#endif
