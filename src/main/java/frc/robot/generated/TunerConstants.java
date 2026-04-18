package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

public class TunerConstants {
    static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(2.66).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

    static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
    static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;
    static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;
    static final Current SLIP_CURRENT = Amps.of(120.0);
    static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS = new TalonFXConfiguration();
    static final TalonFXConfiguration STEER_INITIAL_CONFIGS = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(60.0).withStatorCurrentLimitEnable(true));
    static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS = new CANcoderConfiguration();
    static final Pigeon2Configuration PIGEON_CONFIGS = null;
    static final String CAN_BUS_NAME = "";

    public static final CANBus CAN_BUS = new CANBus("", "./logs/example.hoot");
    public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(4.58);

    static final double COUPLE_RATIO = 3.5714285714285716;
    static final double DRIVE_GEAR_RATIO = 6.746031746031747;
    static final double STEER_GEAR_RATIO = 21.428571428571427;
    static final Distance WHEEL_RADIUS = Inches.of(2.0);

    static final boolean INVERT_LEFT_SIDE = false;
    static final boolean INVERT_RIGHT_SIDE = true;

    static final int PIGEON_ID = 20;

    static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
    static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);

    static final Voltage STEER_FRICTION_VOLTAGE = Volt.of(0.2);
    static final Voltage DRIVE_FRICTION_VOLTAGE = Volt.of(0.2);

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withCANBusName(CAN_BUS_NAME)
            .withPigeon2Id(PIGEON_ID)
            .withPigeon2Configs(PIGEON_CONFIGS);

    static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> CONSTANT_CREATOR =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                    .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                    .withCouplingGearRatio(COUPLE_RATIO)
                    .withWheelRadius(WHEEL_RADIUS)
                    .withSteerMotorGains(STEER_GAINS)
                    .withDriveMotorGains(DRIVE_GAINS)
                    .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                    .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                    .withSlipCurrent(SLIP_CURRENT)
                    .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
                    .withDriveMotorType(DRIVE_MOTOR_TYPE)
                    .withSteerMotorType(STEER_MOTOR_TYPE)
                    .withFeedbackSource(STEER_FEEDBACK_TYPE)
                    .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                    .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                    .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
                    .withSteerInertia(STEER_INERTIA)
                    .withDriveInertia(DRIVE_INERTIA)
                    .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                    .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 15;
    private static final int FRONT_LEFT_ENCODER_ID = 21;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.058837890625;
    private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;
    private static final double FRONT_LEFT_X_POS = 10.875;
    private static final double FRONT_LEFT_Y_POS = 10.875;

    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 13;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 17;
    private static final int FRONT_RIGHT_ENCODER_ID = 23;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.066162109375;
    private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;
    private static final double FRONT_RIGHT_X_POS = 10.875;
    private static final double FRONT_RIGHT_Y_POS = -10.875;

    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 12;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 16;
    private static final int BACK_LEFT_ENCODER_ID = 22;
    private static final double BACK_LEFT_ENCODER_OFFSET = -0.30810546875;
    private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_LEFT_ENCODER_INVERTED = false;
    private static final double BACK_LEFT_X_POS = -10.875;
    private static final double BACK_LEFT_Y_POS = 10.875;

    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 14;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 18;
    private static final int BACK_RIGHT_ENCODER_ID = 24;
    private static final double BACK_RIGHT_ENCODER_OFFSET = -0.42431640625;
    private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;
    private static final double BACK_RIGHT_X_POS = -10.875;
    private static final double BACK_RIGHT_Y_POS = -10.875;

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT =
            CONSTANT_CREATOR.createModuleConstants(
                    FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID, FRONT_LEFT_ENCODER_OFFSET,
                    FRONT_LEFT_X_POS, FRONT_LEFT_Y_POS, INVERT_LEFT_SIDE, FRONT_LEFT_STEER_MOTOR_INVERTED, FRONT_LEFT_ENCODER_INVERTED);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT =
            CONSTANT_CREATOR.createModuleConstants(
                    FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID, FRONT_RIGHT_ENCODER_OFFSET,
                    FRONT_RIGHT_X_POS, FRONT_RIGHT_Y_POS, INVERT_RIGHT_SIDE, FRONT_RIGHT_STEER_MOTOR_INVERTED, FRONT_RIGHT_ENCODER_INVERTED);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT =
            CONSTANT_CREATOR.createModuleConstants(
                    BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ENCODER_OFFSET,
                    BACK_LEFT_X_POS, BACK_LEFT_Y_POS, INVERT_LEFT_SIDE, BACK_LEFT_STEER_MOTOR_INVERTED, BACK_LEFT_ENCODER_INVERTED);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT =
            CONSTANT_CREATOR.createModuleConstants(
                    BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID, BACK_RIGHT_ENCODER_OFFSET,
                    BACK_RIGHT_X_POS, BACK_RIGHT_Y_POS, INVERT_RIGHT_SIDE, BACK_RIGHT_STEER_MOTOR_INVERTED, BACK_RIGHT_ENCODER_INVERTED);

    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
                DRIVETRAIN_CONSTANTS, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
    }

    public static IntakeSubsystem createIntake() {
        return new IntakeSubsystem();
    }

    public static ShooterSubsystem createShooter() {
        return new ShooterSubsystem();
    }

    public static ClimberSubsystem createClimber() {
        return new ClimberSubsystem();
    }

    public static VisionSubsystem createVision() {
        return new VisionSubsystem();
    }
}