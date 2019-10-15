#pragma once

#define LinearMoveSettings PIDSettings(10,0.01,-1)
#define LinearTurnSettings PIDSettings(4,.35,-.05)

#define OdometryMovementGoToSpotTurnPID PIDSettings(4,.35,-.05)
#define OdometryMovementGoToSpotMovePID PIDSettings(5,0.1,-1)

#define TurnTowardsPID PIDSettings(4,.45,-.05)
