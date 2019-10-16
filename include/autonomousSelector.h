#ifndef AUTONOMOUS_SELECTORH
#define AUTONOMOUS_SELECTORH

#include "main.h"

namespace Autonomous{

  enum AutonomousType {Blue = 0, Blue2 = 1, Red = 2, Red2 = 3, Skills = 4, None = 5};

  AutonomousType SelectedAutonomousType;

  void RunSelectedAutonomous();

  char* GetAutonomousTypeString(AutonomousType t);

  void SelectAutonomousType();
}

#endif
