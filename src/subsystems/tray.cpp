#include "main.h"
#include "subsystems/tray.h"
#include "pros/rtos.h"

namespace Subsystems::Tray{

  int target = TrayPosition::Storage;

  pros::Mutex* mutex;

  int GetTarget(){
    mutex->take(10000);
    int t = target;
    mutex->give();
    return t;
  }

  pros::Motor* trayMotor;
  pros::ADIAnalogIn* trayPotentiometer;

  bool init = false;
  bool stopped = false;


  pros::Task* task;
  void RunTray(void* param){
    pros::Task::delay(500);
    while (!stopped){
      int currentPos = trayPotentiometer->get_value();

      int error = currentPos - GetTarget();

      if (abs(error) < 35) error = 0;

      //pros::lcd::print(13, "%f", (float)currentPos);
      //pros::lcd::print(14, "%f", (float)error);

      trayMotor->move_velocity(error*.05);

      pros::Task::delay(50);
    }
  }

  void Init(pros::Motor* trayMtr, pros::ADIAnalogIn* pot){
    trayMotor = trayMtr;
    trayPotentiometer = pot;
    init = true;
    stopped = false;

    task = new pros::Task (RunTray, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
    mutex = new pros::Mutex();
  }

  void MoveTrayToPosition(TrayPosition p){
    mutex->take(10000);
    target = p;
    mutex->give();
  }


}
