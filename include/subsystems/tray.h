#include "main.h"

#pragma once

namespace Subsystems::Tray {

  enum TrayPosition {Storage = 2900, Push = 300};

  void MoveTrayToPosition(TrayPosition);

  void Init(pros::Motor* trayMtr, pros::ADIAnalogIn* pot);
}
