#include "main.h"
#include "subsystems/tray.h"
#include "pros/rtos.h"
int t;
namespace Subsystems::Tray{
  int target = TrayPosition::Storage;

  pros::Mutex* mutex;

  int GetTarget(){
    mutex->take(10000);
    int t = 120;
    if (target == TrayPosition::Storage) {
      t = 120;
    } else if (target == TrayPosition::Stack) {
      t = 2200;
    }
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

      if (abs(error) < 20) error = 0;

      pros::lcd::print(3, "%f", (float)currentPos);
      pros::lcd::print(4, "%f", (float)error);
      pros::lcd::print(5, "TARGET: %f", (float)GetTarget());

    /*  double speedMultiplier = 1;
      if (target == TrayPosition::Stack){
        if (currentPos < 400) speedMultiplier = .2;
        if (currentPos > 0) speedMultiplier = .2;
      }*/
      trayMotor->move_velocity(error*.15/*speedMultiplier*/);

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
