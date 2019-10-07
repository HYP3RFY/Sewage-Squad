#include "main.h"

#ifndef TRAYH
#define TRAYH

namespace Subsystems::Tray {

  enum TrayPosition {Storage = 1322, Push = 1617};

  void MoveTrayToPosition(TrayPosition);

  void Init(pros::Motor* trayMtr, pros::ADIAnalogIn* pot);
}

#endif
