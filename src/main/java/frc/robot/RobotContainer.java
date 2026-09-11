// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  private final Telemetry logger = new Telemetry(MAX_SPEED);

  private VisionSubsystem.VisionTarget hubVisionTarget;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(4.58);
  static final LinearVelocity MAX_SPEED = SPEED_AT_12_VOLTS.times(1.0);
  static final AngularVelocity MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75);

  FieldCentric drive = new FieldCentric()
          .withDeadband(MAX_SPEED.times(0.1))
          .withRotationalDeadband(MAX_ANGULAR_RATE.times(0.1))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  SwerveDriveBrake brake = new SwerveDriveBrake();
  PointWheelsAt point = new PointWheelsAt();

  FieldCentricFacingAngle facingAngle = new FieldCentricFacingAngle()
          .withDeadband(MAX_SPEED.times(0.1))
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE)
          .withRotationalDeadband(MAX_ANGULAR_RATE.times(0.1));
  Angle snapHeading = Degrees.of(0.0);

  private final Autos autos = new Autos(drivetrain, drive, intake, shooter, vision);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joystick =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.facingAngle.HeadingController.setPID(3.0, 0.0, 0.1);
    this.facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // Wire vision subsystem to drivetrain for MegaTag2 + Kalman filter fusion
    this.vision.setDrivetrain(this.drivetrain);

    // Configure autonomous chooser
    this.autoChooser.setDefaultOption(Autos.DO_NOTHING, Autos.DO_NOTHING);
    this.autoChooser.addOption(Autos.DRIVE_FORWARD, Autos.DRIVE_FORWARD);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                    .withVelocityX(MAX_SPEED.times(-joystick.getLeftY()))
                    .withVelocityY(MAX_SPEED.times(-joystick.getLeftX()))
                    .withRotationalRate(MAX_ANGULAR_RATE.times(-joystick.getRightX()))
            )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motor while disabled.
    RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(Idle::new)
                    .ignoringDisable(true)
    );

    // Smart controls
    joystick.back().onTrue(Commands.runOnce(() -> {
      shooter.toggleAdjustableRPM();
      SmartDashboard.putBoolean("Adjustable RPM", shooter.isAdjustableRPMEnabled());
    }));

    // Intake controls

    // Intake jog tuning (Test mode only)
    SmartDashboard.putNumber("Intake/Deploy Jog Step (deg)", IntakeConstants.INTAKE_DEPLOY_JOG_STEP.in(Degrees));
    SmartDashboard.putNumber("Intake/Retract Jog Step (deg)", IntakeConstants.INTAKE_RETRACT_JOG_STEP.in(Degrees));

    joystick.povRight().and(DriverStation::isTestEnabled).onTrue(Commands.runOnce(() -> {
        var step = Degrees.of(Math.abs(SmartDashboard.getNumber("Intake/Deploy Jog Step (deg)",
                IntakeConstants.INTAKE_DEPLOY_JOG_STEP.in(Degrees))));
        this.intake.jogPosition(step);
    }, this.intake));

    joystick.povLeft().and(DriverStation::isTestEnabled).onTrue(Commands.runOnce(() -> {
        var step = Degrees.of(-Math.abs(SmartDashboard.getNumber("Intake/Retract Jog Step (deg)",
                IntakeConstants.INTAKE_RETRACT_JOG_STEP.in(Degrees))));
        this.intake.jogPosition(step);
    }, this.intake));

    this.intake.setDefaultCommand(Commands.run(() -> {
      this.intake.runIntake(0.0);
      this.intake.holdDeployPosition();
    }, this.intake));

    this.joystick.a().whileTrue(Commands.run(() -> this.intake.runIntake(1.0), this.intake));
    this.joystick.b().whileTrue(Commands.run(() -> this.intake.runIntake(-1.0), this.intake));
    this.joystick.x().onTrue(Commands.runOnce(() -> this.intake.setIntakePosition(false), this.intake));
    this.joystick.y().onTrue(Commands.runOnce(() -> this.intake.setIntakePosition(true), this.intake));

    // Shooter controls
    this.shooter.setVision(vision);

    this.shooter.setDefaultCommand(Commands.run(() -> {
      this.shooter.spinUpShooter(0.0);
      this.shooter.setIndexerSpeed(0.0);
    }, this.shooter));

    this.joystick.leftTrigger().whileTrue(Commands.run(() -> {
      double speed = this.joystick.getLeftTriggerAxis();
      this.shooter.spinUpShooter(speed);
    }, this.shooter));

    this.joystick.rightTrigger().whileTrue(Commands.run(() -> shooter.setIndexerSpeed(this.joystick.getRightTriggerAxis()), this.shooter));

    // Snap -to-45: Right stick button locks heading to the nearest corner angle while allowing translation
    this.joystick.rightStick().onTrue(Commands.runOnce(() -> {
      double rawYaw = drivetrain.getPigeon2().getYaw().getValue().magnitude();
      double poseHeading = drivetrain.getState().Pose.getRotation().getDegrees();
      double fieldSnap = Math.round((rawYaw - 45.0) / 90.0) * 90.0 + 45.0;
      this.snapHeading = Degrees.of(fieldSnap - rawYaw + poseHeading);
    }));

    this.joystick.rightStick().whileTrue(drivetrain.applyRequest(() -> facingAngle
            .withVelocityX(MAX_SPEED.times(-joystick.getLeftY()))
            .withVelocityY(MAX_SPEED.times(-joystick.getLeftX()))
            .withTargetDirection(new Rotation2d(this.snapHeading))));

    // Snap-to-hub: D-Pad Up auto rotates to center robot to the Hub AprilTag
    this.joystick.povUp().whileTrue(
            drivetrain.applyRequest(() -> {
              if (vision.hasValidShooterTarget()) {
                var heading = drivetrain.getState().Pose.getRotation().getDegrees();
                var target = heading - vision.getTX();
                return facingAngle
                        .withVelocityX(MAX_SPEED.times(-joystick.getLeftY()))
                        .withVelocityY(MAX_SPEED.times(-joystick.getLeftX()))
                        .withTargetDirection(new Rotation2d(target));
              }
              return facingAngle
                      .withVelocityX(MAX_SPEED.times(-joystick.getLeftY()))
                      .withVelocityY(MAX_SPEED.times(-joystick.getLeftX()))
                      .withTargetDirection(new Rotation2d(drivetrain.getState().Pose.getRotation().getDegrees()));
            })
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    this.joystick.back().and(this.joystick.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    this.joystick.back().and(this.joystick.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    this.joystick.start().and(this.joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    this.joystick.start().and(this.joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // reset the field-centric heading on left bumper press
    this.joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::Telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    var selected = autoChooser.getSelected();

    if (selected.equals(Autos.DRIVE_FORWARD)) {
      return autos.DriveForward();
    }

    // Default: Do Nothing
    return autos.DoNothing();
  }
}
