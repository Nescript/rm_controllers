# rm_controllers

Controller implementations for every robot subsystem, loaded by `controller_manager` according to the robot's config file.

## Language

**Chassis controller**:
Base-class family of controllers that drive the robot's locomotion. Which concrete chassis controller runs is declared by the config's `type:` field.

**Omni controller**:
Chassis controller for omnidirectional chassis — covers BOTH omni-wheel and mecanum-wheel configurations (hero, engineer, standard, spare_sentry all load it).
_Avoid_: mecanum controller, mecanum chassis (mecanum is only a wheel type, not a separate controller)

**Swerve controller**:
Chassis controller for steering-wheel chassis (sentry).

**Bipedal controller**:
Chassis controller for the bipedal-wheel robot, the platform currently under active development by our team (not the team's only robot type). Implemented under `bipedal_wheel_controller/` with VMC-based series-legged control.
_Avoid_: series-legged controller

**Balance controller**:
Chassis controller for inverted-pendulum balance robots. Distinct from the bipedal controller.

**Gimbal controller**:
Stabilizes and aims the yaw/pitch gimbal; includes the bullet solver for trajectory compensation.

**Shooter controller**:
Runs the firing sequence: friction wheels spin up, then the trigger feeds a projectile.

**Friction wheel**:
The pair of high-speed wheels that accelerate a projectile out of the barrel.

**Trigger**:
The mechanism that feeds one projectile into the friction wheels.

**Calibration controller**:
Moves joints to their mechanical zero at startup before normal control begins.

**Orientation controller**:
Broadcasts the chassis orientation as a TF frame, computed from IMU data; also publishes assembly-error diagnostics.
