#include "main.h"

#ifndef TRAYH
#define TRAYH


namespace Subsystems::Tray {

  enum TrayPosition {Storage = 845, Push = 1730, Stack = 1000};

  void MoveTrayToPosition(TrayPosition);

  void Init(pros::Motor* trayMtr, pros::ADIAnalogIn* pot);

}

#endif
