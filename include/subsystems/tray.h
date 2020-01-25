#include "main.h"

#ifndef TRAYH
#define TRAYH


namespace Subsystems::Tray {

  enum TrayPosition {Storage = 1400, Stack = 4000};

  void MoveTrayToPosition(TrayPosition);

  void Init(pros::Motor* trayMtr, pros::ADIAnalogIn* pot);

}

#endif
