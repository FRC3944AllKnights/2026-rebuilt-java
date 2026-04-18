// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public final class Autos {
  public static final String DO_NOTHING = "Do Nothing";
  public static final String DRIVE_FORWARD = "Drive Forward";
  public static final String BACK_UP_AND_SHOOT = "Back Up and Shoot";

  CommandSwerveDrivetrain drivetrain;
  FieldCentric drive;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  VisionSubsystem vision;

  public Autos(CommandSwerveDrivetrain drivetrain, FieldCentric drive, IntakeSubsystem intake, ShooterSubsystem shooter, VisionSubsystem vision) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.intake = intake;
    this.shooter = shooter;
    this.vision = vision;
  }

  public Command DoNothing() {
    return Commands.none();
  }

  public Command DriveForward() {
    return Commands.sequence(
            this.drivetrain.runOnce(() -> drivetrain.seedFieldCentric(new Rotation2d(0.0))),
            this.drivetrain.applyRequest(() ->
                    drive.withVelocityX(0.5)
                      .withVelocityY(0.0)
                      .withRotationalRate(0.0))
                    .withTimeout(5.0),
            drivetrain.applyRequest(SwerveRequest.Idle::new)
    );
  }

  public Command BackUpAndShoot() {
    return Commands.sequence(
            this.drivetrain.runOnce(() -> drivetrain.seedFieldCentric(new Rotation2d(0.0))),
            this.drivetrain.applyRequest(() ->
                    drive.withVelocityX(-0.5)
                      .withVelocityY(0.0)
                      .withRotationalRate(0.0))
                    .withTimeout(2.0),
            drivetrain.applyRequest(SwerveRequest.Idle::new),
            this.shooter.run(() -> shooter.spinUpShooter(1.0)).withTimeout(2.0),
            shooter.run(() -> {
              this.shooter.run(() -> shooter.spinUpShooter(1.0));
              this.shooter.run(() -> shooter.setIndexerSpeed(1.0));
            }).withTimeout(6.0),

            this.shooter.runOnce(() -> {
              shooter.setIndexerSpeed(0.0);
              shooter.spinUpShooter(0.0);
            })
    );
  }
}
