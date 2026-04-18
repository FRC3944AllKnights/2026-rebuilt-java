# WPILib Cheat Sheet

Quick reference for the WPILib classes and methods we use most in this codebase. Every snippet below is pulled from our actual robot code.

---

## Commands

Commands are how you make the robot do things. You can chain them together, add timeouts, and bind them to buttons.

### `Commands.run(action, subsystem)`

Runs an action **repeatedly** (every 20ms) until the command is cancelled.

```java
// Spin intake rollers while A button is held
joystick.a().whileTrue(
    Commands.run(() -> this.intake.runIntake(1.0), this.intake)
);
```

**When you'd use this:** Any action that needs to keep running — driving, spinning motors, holding a position.

---

### `Commands.runOnce(action, subsystem)`

Runs an action **exactly once** and finishes immediately.

```java
// Toggle adjustable RPM when Back button is pressed
joystick.back().onTrue(Commands.runOnce(() -> {
    shooter.toggleAdjustableRPM();
}));
```

**When you'd use this:** Toggling a setting, resetting a sensor, one-time actions.

---

### `Commands.sequence(cmd1, cmd2, cmd3...)`

Runs commands **one after another**. Each one must finish before the next starts.

```java
return Commands.sequence(
    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(new Rotation2d(0.0))),
    drivetrain.applyRequest(() -> drive.withVelocityX(0.5)).withTimeout(5.0),
    drivetrain.applyRequest(SwerveRequest.Idle::new)
);
```

**When you'd use this:** Autonomous routines, any multi-step action.

---

### `command.withTimeout(seconds)`

Adds a time limit to a command. The command stops after the given number of seconds even if it hasn't finished naturally.

```java
// Drive forward for at most 5 seconds
drivetrain.applyRequest(() -> drive.withVelocityX(0.5))
    .withTimeout(5.0)
```

**When you'd use this:** Autonomous steps, safety limits on any timed action.

---

### `Commands.waitUntil(condition)`

Waits (does nothing) until a condition becomes true.

```java
// Wait for the intake arm to reach its target position
waitUntil(this::isAtPosition).withTimeout(2)
```

**When you'd use this:** Waiting for a mechanism to reach position before proceeding, often paired with `.withTimeout()` as a safety fallback.

---

### `Commands.none()`

A command that does absolutely nothing and finishes immediately.

```java
public Command DoNothing() {
    return Commands.none();
}
```

**When you'd use this:** Default option in the autonomous chooser when you don't want the robot to move.

---

## Controller Input

### `CommandXboxController`

Wraps an Xbox controller and gives you `Trigger` objects for each button.

```java
private final CommandXboxController joystick =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
```

**Joystick axes:** `getLeftY()`, `getLeftX()`, `getRightX()`, `getLeftTriggerAxis()`, `getRightTriggerAxis()`

**Button triggers:** `a()`, `b()`, `x()`, `y()`, `leftBumper()`, `rightBumper()`, `back()`, `start()`, `leftStick()`, `rightStick()`, `povUp()`, `povDown()`, `povLeft()`, `povRight()`, `leftTrigger()`, `rightTrigger()`

---

### `Trigger` — `.onTrue()`, `.whileTrue()`, `.onFalse()`

Triggers let you bind commands to button events.

| Method | Behavior |
|--------|----------|
| `.onTrue(cmd)` | Run the command **once** when the button is first pressed |
| `.whileTrue(cmd)` | Run the command **as long as** the button is held, cancel on release |
| `.onFalse(cmd)` | Run the command **once** when the button is released |

```java
// onTrue: Toggle a setting on press
joystick.rightBumper().onTrue(
    Commands.runOnce(() -> drivetrain.toggleAutoXBraking())
);
```

---

### `MathUtil.applyDeadband(value, deadband)`

Filters out small joystick values near center. Returns 0.0 if the input is within the deadband range.

```java
// Ignore stick movement smaller than 10%
MathUtil.applyDeadband(joystick.getLeftY(), 0.1)
```

**When you'd use this:** Always use on joystick inputs to prevent robot drift when sticks aren't perfectly centered.

---

## SmartDashboard

### `SmartDashboard.putNumber(key, value)` / `putBoolean(key, value)`

Sends a value to the dashboard so you can see it on the driver station laptop.

```java
SmartDashboard.putNumber("Shooter Wheel Target RPM", targetRPM.magnitude());
SmartDashboard.putBoolean("Adjustable RPM", shooter.isAdjustableRPMEnabled());
```

**When you'd use this:** Debugging, monitoring sensor values, displaying robot state during matches.

---

### `SmartDashboard.getNumber(key, defaultValue)`

Reads a value back from the dashboard (useful for live tuning).

```java
var step = Rotations.of(
    SmartDashboard.getNumber("Intake/Deploy Jog Step (tr)", 5.0)
);
```

**When you'd use this:** Letting the drive team adjust a value on the fly without redeploying code.

---

### `SmartDashboard.putData(key, sendable)`

Puts a complex object on the dashboard. Used for the autonomous chooser.

```java
SmartDashboard.putData("Auto Chooser", autoChooser);
```

---

## Geometry & Units

### `Rotation2d`

Represents an angle. Used for headings, field-centric direction, and target angles.

```java
drivetrain.seedFieldCentric(new Rotation2d(0.0));
```

---

### `Pose2d`

Represents a position + heading on the field (x, y, rotation). The drivetrain tracks this.

```java
double heading = drivetrain.getState().Pose.getRotation().getDegrees();
```

---

## Utilities

### `SendableChooser`

A dropdown menu on the dashboard. We use it to pick autonomous routines.

```java
private final SendableChooser<String> autoChooser = new SendableChooser<>();
autoChooser.setDefaultOption(Autos.DO_NOTHING, Autos.DO_NOTHING);
autoChooser.addOption(Autos.DRIVE_FORWARD, Autos.DRIVE_FORWARD);
```

---

### `RobotModeTriggers`

Triggers that fire when the robot enters a specific mode (auto, teleop, test, disabled).

```java
// Auto-prime the intake when autonomous or teleop starts
RobotModeTriggers.autonomous().onTrue(intake.primeIntakeCommand());
RobotModeTriggers.teleop().onTrue(intake.primeIntakeCommand());
```

**When you'd use this:** Running setup commands at the start of a match phase.

---

### `AddressableLED` / `LEDPattern`

Controls RGB LED strips. See [Subsystems Guide — LEDs](subsystems.md#leds) for details.

```java
LEDPattern pattern = LEDPattern.solid(Color.kYellow);
pattern.applyTo(this.ledBuffer);
this.led.setData(this.ledBuffer);
```
