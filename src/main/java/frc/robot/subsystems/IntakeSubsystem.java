package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Telemetry;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFXS intakeDeployLeftMotor = new TalonFXS(CANConstants.INTAKE_DEPLOY_LEFT_MOTOR_ID, CANConstants.CAN_BUS);
    private final TalonFXS intakeRollerMotor = new TalonFXS(CANConstants.INTAKE_ROLLER_MOTOR_ID, CANConstants.CAN_BUS);

    private Angle targetPosition = IntakeConstants.INTAKE_START_POSITION;

    public IntakeSubsystem() {
        // Configure deploy arm motors
        // Left motor
        var deployLeftConfig = new TalonFXSConfiguration();

        // Set motor type to NEO 550s
        deployLeftConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        // Set slot 0 constants
        deployLeftConfig.Slot0.kP = IntakeConstants.INTAKE_DEPLOY_P;
        deployLeftConfig.Slot0.kI = IntakeConstants.INTAKE_DEPLOY_I;
        deployLeftConfig.Slot0.kD = IntakeConstants.INTAKE_DEPLOY_D;
        deployLeftConfig.Slot0.kS = IntakeConstants.INTAKE_DEPLOY_S;
        deployLeftConfig.Slot0.kV = IntakeConstants.INTAKE_DEPLOY_V;
        deployLeftConfig.Slot0.kG = IntakeConstants.INTAKE_DEPLOY_G;

        // Set motion magic variables
        deployLeftConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.INTAKE_DEPLOY_CRUISE_VELOCITY;
        deployLeftConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.INTAKE_DEPLOY_ACCELERATION;
        deployLeftConfig.MotionMagic.MotionMagicJerk = IntakeConstants.INTAKE_DEPLOY_JERK;

        // Set current limits
        deployLeftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployLeftConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_DEPLOY_SUPPLY_CURRENT_LIMIT;

        // Set gear reduction
        deployLeftConfig.ExternalFeedback.SensorToMechanismRatio = IntakeConstants.INTAKE_DEPLOY_GEAR_RATIO;

        // Set software limits
        deployLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        deployLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        deployLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.INTAKE_DEPLOYED_POSITION.magnitude();
        deployLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.INTAKE_START_POSITION.magnitude();

        // Set brake on boot and invert motor
        deployLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        this.intakeDeployLeftMotor.getConfigurator().apply(deployLeftConfig);

        // Right motor
        var deployRightConfig = new TalonFXSConfiguration();

        // Set motor type to NEO 550s
        deployRightConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        // Set current limits
        deployRightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployRightConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_DEPLOY_SUPPLY_CURRENT_LIMIT;

        // Configure roller motors
        var rollerConfig = new TalonFXSConfiguration();

        // Set motor type to NEOs
        rollerConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

        // Set current limits
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_ROLLERS_SUPPLY_CURRENT_LIMIT;

        this.intakeRollerMotor.getConfigurator().apply(rollerConfig);

        // Set encoder to 0, the current starting position
        this.intakeDeployLeftMotor.setPosition(IntakeConstants.INTAKE_START_POSITION);
    }

    public void runIntake(double speed) {
        // Power intake motors
        // speed: [-1.0, 1.0] for reverse to forward
        // DutyCycleOut sends a percentage of available voltage - no feedback loop

        intakeRollerMotor.setControl(new DutyCycleOut(speed));
    }

    public void setIntakePosition(boolean up) {
        // Set intake positions using Motion Magic - smooth, profiled position control
        // up: true = retracted (stowed), false = deployed (down to collect game pieces)
        // MotionMagicVoltage tells the motor controller to go to an exact position
        // following the trapezoidal/S-curve profile configured in the constructor

        var targetPosition = up
                ? IntakeConstants.INTAKE_START_POSITION
                : IntakeConstants.INTAKE_DEPLOYED_POSITION;

        this.targetPosition = targetPosition;

        this.intakeDeployLeftMotor.setControl(new MotionMagicVoltage(targetPosition));
    }

    public void setDeployTarget(Angle position) {
        this.targetPosition = position;

        this.intakeDeployLeftMotor.setControl(new MotionMagicVoltage(position));
    }

    public void holdDeployPosition() {
        this.intakeDeployLeftMotor.setControl(new MotionMagicVoltage(this.targetPosition));
    }

    public boolean isAtPosition() {
        var currentPosition = getCurrentPosition();
        return currentPosition.minus(this.targetPosition).abs(Rotations) < IntakeConstants.INTAKE_POSITION_TOLERANCE;
    }

    public Angle getCurrentPosition() {
        return this.intakeDeployLeftMotor.getPosition().getValue();
    }

    public Angle getTargetPosition() {
        return this.targetPosition;
    }

    public void jogPosition(Angle steps) {
        var targetPosition = this.targetPosition.plus(steps);

        this.intakeDeployLeftMotor.setControl(new MotionMagicVoltage(targetPosition));
    }

    public void publishTelemetry() {
        SmartDashboard.putNumber("Intake/Current Position (tr)", intakeDeployLeftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Target Position (tr)", targetPosition.magnitude());
    }
}
