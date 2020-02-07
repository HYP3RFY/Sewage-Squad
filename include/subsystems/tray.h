#include "main.h"

#ifndef TRAYH
#define TRAYH


namespace Subsystems::Tray {

  enum TrayPosition {Storage = 122, Stack = 2200};

  void MoveTrayToPosition(TrayPosition);

  void Init(pros::Motor* trayMtr, pros::ADIAnalogIn* pot);

}

#endif
