#include "main.h"
#include "autonomousSelector.h"

namespace Autonomous{
  void RunSelectedAutonomous(){
  }

  char* GetAutonomousTypeString(AutonomousType t){
    switch(t){
      case(AutonomousType::Blue): return "Blue";
      case(AutonomousType::Blue2): return "Blue2";
      case(AutonomousType::Red): return "Red";
      case(AutonomousType::Red2): return "Red2";
      case(AutonomousType::Skills): return "Skills";
      case(AutonomousType::None): return "None";
    }
  }

  void SelectAutonomousType(){
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::delay(50);
    while(true){
      pros::delay(100);
      pros::lcd::print(6, "%s", Autonomous::GetAutonomousTypeString(Autonomous::SelectedAutonomousType));
      if (master.get_digital_new_press(DIGITAL_Y)){
          Autonomous::SelectedAutonomousType = (Autonomous::AutonomousType)(((int)Autonomous::SelectedAutonomousType+1)%6);
        }
    }
  }
}
