# Team 3944 AllKnights — 2026 Robot Code Documentation

Welcome! This is the documentation for our 2026 FRC robot code. It's written for **everyone on the team**, whether you're brand new to programming or you've been coding for years.

## What Does Our Robot Do?

Our robot is built for the 2026 FRC game. It can:

- **Drive** using a swerve drivetrain (all 4 wheels can steer independently — it can move in any direction)
- **Intake** game pieces from the ground using a deployable arm with rollers
- **Shoot** game pieces into the hub using a dual-motor shooter with automatic speed adjustment
- **See** the field using a Limelight camera that tracks AprilTag targets for aiming and position tracking
- **Light up** with an LED strip that shows the robot's status

## Documentation Guide

Start with the **Project Overview** to understand how the code is structured, then read about whichever subsystem you're working on.

| Document | What It Covers |
|----------|---------------|
| [Project Overview](project-overview.md) | How the code is organized, what each file does, how commands work |
| [Subsystems Guide](subsystems.md) | How each physical part of the robot works in code (drivetrain, intake, shooter, vision, LEDs) |
| [WPILib Cheat Sheet](wpilib-cheatsheet.md) | Quick reference for the most common WPILib calls we use |
| [Phoenix 6 Cheat Sheet](phoenix6-cheatsheet.md) | Quick reference for CTRE motor controller calls we use |

## Glossary

These terms come up all the time in FRC programming. Here's what they mean in plain English:

| Term | What It Means |
|------|--------------|
| **Subsystem** | A class that represents one physical part of the robot (like the drivetrain or shooter). It "owns" the motors and sensors for that part. |
| **Command** | An action the robot performs, like "drive forward" or "spin up the shooter." Commands use subsystems to do their work. |
| **CommandScheduler** | The brain of the robot — it runs every 20ms and decides which commands to execute. You almost never touch this directly. |
| **CAN Bus** | The wire network that connects the RoboRIO to all our motor controllers and sensors. Each device has a unique ID number. |
| **CAN ID** | The unique number assigned to each motor or sensor on the CAN bus. Ours are defined in `Constants.java`. |
| **PID** | A control algorithm that automatically adjusts motor output to reach a target (like a specific RPM or position). P = how hard it tries, I = fixes small steady errors, D = prevents overshooting. |
| **Odometry** | The robot's estimate of where it is on the field, calculated from wheel encoder data and the gyro. |
| **Swerve Drive** | A drivetrain where each wheel can rotate independently, allowing the robot to drive in any direction while facing any direction. |
| **Field-Centric** | A driving mode where "forward" always means "toward the other alliance wall," regardless of which way the robot is facing. Uses the gyro to achieve this. |
| **AprilTag** | A black-and-white square barcode printed on the field walls. Our Limelight camera detects these to figure out where we are and where to aim. |
| **Limelight** | Our vision camera. It processes images and sends targeting data (angles, distances) to the RoboRIO over the network. |
| **SmartDashboard** | A tool on the driver station laptop that shows live values from the robot and lets you tweak settings in real time. |
| **Deadband** | A small zone around the joystick center that's treated as "zero." Prevents the robot from drifting when you let go of the stick. Ours is set to 10%. |
| **TalonFX / TalonFXS** | The motor controllers we use (made by CTRE). They run the motors and handle PID math onboard. TalonFXS supports external motors like NEOs. |
| **MotionMagic** | A CTRE control mode that moves a motor to a position following a smooth S-curve profile instead of jerking to the position instantly. |
| **Duty Cycle** | A simple control mode that sends a percentage of available voltage to a motor. 1.0 = full power, 0.5 = half power, 0.0 = off. |
