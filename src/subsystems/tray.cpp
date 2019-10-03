#include "main.h"
#include "subsystems/tray.h"

namespace Subsystems::Tray{

  TrayPosition target = TrayPosition::Storage;

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


      trayMotor->move_velocity(error*.2);

      pros::Task::delay(50);
    }
  }

  void Init(pros::Motor* trayMtr, pros::ADIAnalogIn* pot){
    trayMotor = trayMtr;
    trayPotentiometer = pot;
    init = true;
    stopped = false;

    task = new pros::Task (RunTray);
    mutex = new pros::Mutex();
  }

  void MoveTrayToPosition(TrayPosition p){
    mutex->take(10000);
    target = p;
    mutex->give();
  }


}
