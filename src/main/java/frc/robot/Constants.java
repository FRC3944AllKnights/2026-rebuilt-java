// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AprilTagConstants {
    // -- Red Hub (central hex structure) --
    public static final int RED_HUB_NORTH_CENTER = 5;
    public static final int RED_HUB_NORTH_OFFSET = 8;
    public static final int RED_HUB_SOUTH_CENTER = 11;
    public static final int RED_HUB_SOUTH_OFFSET = 2;
    public static final int RED_HUB_WEST_CENTER = 9;
    public static final int RED_HUB_WEST_OFFSET = 10;
    public static final int RED_HUB_EAST_CENTER = 3;
    public static final int RED_HUB_EAST_OFFSET = 4;

    // -- Blue Hub --
    public static final int BLUE_HUB_NORTH_CENTER = 27;
    public static final int BLUE_HUB_NORTH_OFFSET = 18;
    public static final int BLUE_HUB_SOUTH_CENTER = 21;
    public static final int BLUE_HUB_SOUTH_OFFSET = 24;
    public static final int BLUE_HUB_WEST_CENTER = 19;
    public static final int BLUE_HUB_WEST_OFFSET = 20;
    public static final int BLUE_HUB_EAST_CENTER = 25;
    public static final int BLUE_HUB_EAST_OFFSET = 26;

    // -- Red Trench (above/below hub) --
    public static final int RED_TRENCH_NORTH_CENTER = 7;
    public static final int RED_TRENCH_NORTH_OFFSET = 6;
    public static final int RED_TRENCH_SOUTH_CENTER = 1;
    public static final int RED_TRENCH_SOUTH_OFFSET = 12;

    // -- Blue Trench --
    public static final int BLUE_TRENCH_NORTH_CENTER = 17;
    public static final int BLUE_TRENCH_NORTH_OFFSET = 28;
    public static final int BLUE_TRENCH_SOUTH_CENTER = 23;
    public static final int BLUE_TRENCH_SOUTH_OFFSET = 22;

    // -- Red Outpost --
    public static final int RED_OUTPOST_CENTER = 13;
    public static final int RED_OUTPOST_OFFSET = 14;

    // -- Blue Outpost --
    public static final int BLUE_OUTPOST_CENTER = 29;
    public static final int BLUE_OUTPOST_OFFSET = 30;

    // -- Red Tower --
    public static final int RED_TOWER_CENTER = 15;
    public static final int RED_TOWER_OFFSET = 16;

    // -- Blue Tower --
    public static final int BLUE_TOWER_CENTER = 31;
    public static final int BLUE_TOWER_OFFSET = 32;

    public static final int RED_SHOOTER_TAG_ID = 9;
    public static final int BLUE_SHOOTER_TAG_ID = 25;
  }

  public static class IntakeConstants {
    public static final double INTAKE_DEPLOY_GEAR_RATIO = 12.8;

    public static final double INTAKE_DEPLOY_P = 0.1;
    public static final double INTAKE_DEPLOY_I = 0.0;
    public static final double INTAKE_DEPLOY_D = 0.0;

    public static final double INTAKE_DEPLOY_S = 0.25;
    public static final double INTAKE_DEPLOY_V = 0.12;
    public static final double INTAKE_DEPLOY_G = 0.1;

    public static final double INTAKE_DEPLOY_CRUISE_VELOCITY = 5.0;
    public static final double INTAKE_DEPLOY_ACCELERATION = 10.0;
    public static final double INTAKE_DEPLOY_JERK = 100.0;

    public static final Angle INTAKE_DEPLOYED_POSITION = Rotations.of(50.0);
    public static final Angle INTAKE_START_POSITION = Rotations.of(0.0);

    public static final double INTAKE_DEPLOY_SUPPLY_CURRENT_LIMIT = 20.0;
    public static final double INTAKE_ROLLERS_SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double INTAKE_POSITION_TOLERANCE = 0.5;

    public static final Angle INTAKE_DEPLOY_JOG_STEP = Rotations.of(5.0);
    public static final Angle INTAKE_RETRACT_JOG_STEP = Rotations.of(0.5);
  }

  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight-intake";
  }

  public static class ShooterConstants {
    public static final Distance SHOOTER_CIRCUMFERENCE = Inches.of(12.56);
    public static final Angle EXIT_ANGLE = Degrees.of(45.0);
    public static final Distance SHOOTER_HEIGHT = Inches.of(20.0);
    public static final Distance SHOOTER_OFFSET_FROM_REAR_BUMPER = Inches.of(10.0);
    public static final Distance FORWARD_CAMERA_HEIGHT = Inches.of(18.0);
    public static final Distance FORWARD_CAMERA_SHOOTER_OFFSET = Inches.of(0.0);
    public static final Angle FORWARD_CAMERA_ANGLE = Degrees.of(15);

    public static final Distance HUB_HEIGHT = Inches.of(72.0);
    public static final Distance HUB_TAG_HEIGHT = Inches.of(44.25);
    public static final Distance HUB_WIDTH_MAX = Inches.of(44.4);

    public static final LinearAcceleration GRAVITATIONAL_ACCELERATION = InchesPerSecondPerSecond.of(386.2205);

    public static final double SHOOTER_BALL_SPEED_TRANSFER_PERCENT = 0.46;
    public static final AngularVelocity TARGET_SHOOTER_BASE_SPEED = RotationsPerSecond.of(8.3333);

    public static final Distance BASE_RANGE = Inches.of(136.46);
    public static final Distance MINIMUM_RANGE = Inches.of(84.0);
    public static final int LED_PORT = 1;
    public static final int NUMBER_OF_LEDS = 60;

    public static final double SHOOTER_P = 0.5;
    public static final double SHOOTER_I = 0.5;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_S = 0.15;
    public static final double SHOOTER_V = 0.127;

    public static final double SHOOTER_SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double INDEXER_SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double INDEXER_P = 0.1;
    public static final double INDEXER_I = 0.0;
    public static final double INDEXER_D = 0.0;

    public static final boolean DEBUG_PRINTS_ENABLED = true;
  }

  public static class CANConstants {

    // CAN Bus
    public static final CANBus CAN_BUS = new CANBus("rio");

    // Robot Core
    public static final int PDP_ID = 0;
    public static final int RIO_ID = 1;

    // Drivetrain Motors
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 12;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 13;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 14;

    public static final int FRONT_LEFT_STEER_MOTOR_ID = 15;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 16;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 17;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 18;

    // Drivetrain Sensors
    public static final int PIGEON_IMU_ID = 20;
    public static final int FRONT_LEFT_ENCODER_ID = 21;
    public static final int BACK_LEFT_ENCODER_ID = 22;
    public static final int FRONT_RIGHT_ENCODER_ID = 23;
    public static final int BACK_RIGHT_ENCODER_ID = 24;

    // Intake
    public static final int INTAKE_DEPLOY_LEFT_MOTOR_ID = 30;
    public static final int INTAKE_ROLLER_MOTOR_ID = 32;

    // Shooter
    public static final int SHOOTER_LEFT_MOTOR_ID = 40;
    public static final int SHOOTER_RIGHT_MOTOR_ID = 41;
    public static final int INDEXER_LEFT_MOTOR_ID = 42;

    // Climber
    //static final int CLIMBER_MOTOR_ID = 50;
  }
}
