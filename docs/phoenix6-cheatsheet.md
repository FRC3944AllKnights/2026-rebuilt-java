# Phoenix 6 Cheat Sheet

Quick reference for the CTRE Phoenix 6 classes and methods we use to control our motors. Every snippet below is from our actual robot code.

> **Phoenix 6** is the library from CTRE (Cross The Road Electronics) that controls TalonFX and TalonFXS motor controllers.

---

## Motor Types

| Class | What It Is | Where We Use It |
|-------|-----------|-----------------|
| `TalonFX` | Motor controller with a built-in Falcon 500 motor | Drivetrain (drive & steer motors) |
| `TalonFXS` | Motor controller that works with external motors (NEO, NEO 550) | Intake, shooter, indexer |

### Creating a Motor

```java
TalonFXS shooterLeftMotor = new TalonFXS(
    CANConstants.SHOOTER_LEFT_MOTOR_ID, CANConstants.CAN_BUS
);
```

The two arguments are the **CAN ID** (a number from `Constants.java`) and which **CAN bus** to use.

---

## Configuring a Motor

Before using a motor, you configure it with a `TalonFXSConfiguration` object. This sets up everything: motor type, PID values, current limits, gear ratios, etc.

### Basic Configuration Pattern

```java
var config = new TalonFXSConfiguration();
config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;
config.CurrentLimits.SupplyCurrentLimitEnable = true;
config.CurrentLimits.SupplyCurrentLimit = 40.0;
this.motor.getConfigurator().apply(config);
```

This pattern is always the same: create config → set values → apply.

### Motor Type

Tells the controller what kind of motor is connected:

```java
// For NEO motors (full size)
config.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

// For NEO 550 motors (small)
config.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
```

### Current Limits

Prevents motors from drawing too much power and tripping breakers:

```java
config.CurrentLimits.SupplyCurrentLimitEnable = true;
config.CurrentLimits.SupplyCurrentLimit = 40.0; // amps
```

### Brake vs Coast

Controls what happens when you stop commanding the motor:

```java
// Brake: motor resists being moved (like a parking brake)
config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
```

### Gear Ratio

Tells the controller the gear ratio so position/velocity readings are in mechanism units (not raw motor rotations):

```java
config.ExternalFeedback.SensorToMechanismRatio = 12.8;
```

---

## Control Modes

Control modes tell the motor **how** to move. We use four different modes in our robot.

### `DutyCycleOut` — Simple Power %

Sends a percentage of available voltage. No feedback loop — if the motor slows down under load, it stays at the same voltage.

```java
// Run intake roller at full speed
intakeRollerMotor.setControl(new DutyCycleOut(1.0));

// Stop
intakeRollerMotor.setControl(new DutyCycleOut(0.0));
```

| Value | Effect |
|-------|--------|
| `1.0` | Full power forward |
| `0.0` | Stop |
| `-1.0` | Full power reverse |

**When we use this:** Intake rollers, indexer — anything where exact speed doesn't matter, just "on" or "off."

---

### `VelocityVoltage` — Target RPM

The motor controller uses PID to maintain a specific speed. If something pushes against the motor, it increases voltage to compensate.

```java
AngularVelocity targetSpeed = RevolutionsPerSecond.of(44.16);
VelocityVoltage request = new VelocityVoltage(targetSpeed);
shooterLeftMotor.setControl(request.withSlot(0));
```

`.withSlot(0)` tells it which set of PID values to use (configured in `Slot0` during setup).

**When we use this:** Shooter flywheels — we need a precise RPM to control shot distance.

---

### `MotionMagicVoltage` — Smooth Position

Moves the motor to an exact position following a smooth **S-curve** profile. Instead of slamming to the target, it accelerates smoothly, cruises, then decelerates.

```java
// Move intake arm to the deployed position
intakeDeployLeftMotor.setControl(
    new MotionMagicVoltage(IntakeConstants.INTAKE_DEPLOYED_POSITION)
);
```

The profile is configured during motor setup:

```java
config.MotionMagic.MotionMagicCruiseVelocity = 5.0;   // max speed
config.MotionMagic.MotionMagicAcceleration = 10.0;     // how fast it speeds up
config.MotionMagic.MotionMagicJerk = 100.0;            // smoothness of acceleration
```

**When we use this:** Intake deploy arm — smooth movement prevents mechanical stress and game piece drops.

---

### `Follower` — Mirror Another Motor

Makes one motor copy another motor's output exactly. Used when two motors drive the same mechanism from opposite sides.

```java
// Right motor follows left motor (inverted because they're on opposite sides)
intakeDeployRightMotor.setControl(
    new Follower(intakeDeployLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed)
);
```

`MotorAlignmentValue.Opposed` means the follower spins in the **opposite direction** — needed when motors face each other across a mechanism.

**When we use this:** Intake deploy (left + right), shooter (left + right), indexer (left + right).

---

### `NeutralOut` — Stop

Tells the motor to go to its neutral state (brake or coast depending on config).

```java
shooterLeftMotor.setControl(new NeutralOut());
```

---

## Reading Sensor Values

Every TalonFX/TalonFXS has built-in encoders. You can read position, velocity, and voltage.

### Position (rotations)

```java
// Get current position of the intake arm
Angle position = intakeDeployLeftMotor.getPosition().getValue();
```

### Velocity (rotations per second)

```java
// Get current shooter wheel speed
var rpm = shooterLeftMotor.getRotorVelocity().getValue().times(60.0);
```

### Supply Voltage

```java
var voltage = shooterLeftMotor.getSupplyVoltage().getValueAsDouble();
```

### Setting Position (reset encoder)

```java
// Tell the motor "you are currently at position 0"
intakeDeployLeftMotor.setPosition(IntakeConstants.INTAKE_START_POSITION);
```

---

## PID Basics

PID values control how the motor reaches its target. They're set during configuration.

```java
config.Slot0.kP = 0.5;   // Proportional
config.Slot0.kI = 0.5;   // Integral
config.Slot0.kD = 0.0;   // Derivative
config.Slot0.kS = 0.15;  // Static friction feedforward
config.Slot0.kV = 0.127; // Velocity feedforward
```

### What Each Value Does

| Value | Plain English | Too High | Too Low |
|-------|--------------|----------|---------|
| **kP** | How aggressively it corrects errors. "I'm 10 RPM off, so push harder." | Oscillates / overshoots | Sluggish, never reaches target |
| **kI** | Fixes small steady errors over time. "I've been 2 RPM off for a while, nudge a bit more." | Wind-up oscillation | Small constant error remains |
| **kD** | Slows down as it approaches the target. "I'm getting close, ease off." | Jittery / vibrating | Overshoots before settling |
| **kS** | Minimum voltage to overcome friction (motor doesn't move below this). | Jumps too hard at start | Doesn't start moving |
| **kV** | Voltage per unit of velocity. "To spin at X speed, I need roughly Y volts." | Runs too fast | Runs too slow |
| **kG** | Gravity compensation. Adds extra voltage to hold position against gravity. | Drifts up | Drifts down |

> **Tip:** When tuning, start with **kP** only (set everything else to 0). Get it close, then add **kV** for velocity or **kG** for position control. Only add **kI** and **kD** if needed.

---

## Swerve-Specific (Drivetrain)

The drivetrain uses Phoenix 6's built-in swerve library. Most of this is auto-generated in `TunerConstants.java`, but here are the key concepts:

### SwerveRequest

A `SwerveRequest` tells the drivetrain **what to do**. You apply it with `setControl()`:

```java
// Apply a field-centric drive request
drivetrain.setControl(
    drive.withVelocityX(speed).withVelocityY(strafe).withRotationalRate(rotation)
);
```

### CANcoder

Each swerve module has a CANcoder that tracks the **absolute angle** of the wheel. This is how the robot knows which direction each wheel is pointed, even after a restart.

### Pigeon2 IMU

The gyro/accelerometer mounted at the center of the robot. It tracks the robot's heading (which way it's facing) for field-centric driving.

```java
// Read the current yaw (heading) in degrees
var yaw = drivetrain.getPigeon2().getYaw().getValue();
```
