#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "main.h"

/*
  sL = left distance from tracking center
  sR = right distance from tracking center
  sS = back distance from tracking center
*/

namespace Odometry{
  struct Vector2{
  private:
    double x;
    double y;
  public:
    double getX();
    void setX(double newX);
    double getY();
    void setY(double newY);

    double getTheta(); //In radians
    double getMagnitude();

    Vector2 normalize();

    static Vector2 FromPolar(double theta, double r);
    static double Distance(Vector2 a, Vector2 b);

    Vector2(double x, double y);

    Vector2 operator+(Vector2 other);
    Vector2 operator-(Vector2 other);
    Vector2 operator-();
    Vector2 operator*(double other);

    Vector2 RotateBy(double theta);
    Vector2 Transpose();
     std::string ToString();
  };

  void SetTrackingCenterParameters(double sL, double sR, double sS); // should be called before init
  void SetStartingPositionAndRotation(Vector2 pos, double rotation); //should be called before init, otherwise default to 0
  void SetTrackingWheelEncoders(pros::ADIEncoder* leftWheel, pros::ADIEncoder* rightWheel, pros::ADIEncoder* backWheel); //MUST BE CALLED BEFORE INIT
  void SetWheelCircumference(double c); //defaults to circumference of pi
  void Init(); //Spawns a thread that runs the tracking algorithm
  void Stop(); //stops the thread that runs the tracking algorithm
  Vector2 GetRobotPosition(); //Gets the robot's absolute position
  double GetRobotRotation(); //Gets the robot's absolute rotation

}

#endif
