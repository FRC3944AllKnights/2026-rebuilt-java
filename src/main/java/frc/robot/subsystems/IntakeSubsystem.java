package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFXS intakeDeployLeftMotor = new TalonFXS(CANConstants.INTAKE_DEPLOY_LEFT_MOTOR_ID, CANConstants.CAN_BUS);
    private final TalonFXS intakeRollerMotor = new TalonFXS(CANConstants.INTAKE_ROLLER_MOTOR_ID, CANConstants.CAN_BUS);

    private Angle targetPosition = IntakeConstants.INTAKE_START_POSITION;

    public IntakeSubsystem() {
        // Configure the single deploy arm motor (CAN 30).
        var deployLeftConfig = new TalonFXSConfiguration();

        // Deploy motor: full-size NEO (REV-21-1650), with JST sensor connection.
        deployLeftConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

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
        deployLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        deployLeftConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_DEPLOY_STATOR_CURRENT_LIMIT;
        deployLeftConfig.Voltage.PeakForwardVoltage = IntakeConstants.INTAKE_DEPLOY_MAX_VOLTAGE;
        deployLeftConfig.Voltage.PeakReverseVoltage = -IntakeConstants.INTAKE_DEPLOY_MAX_VOLTAGE;

        // Set gear reduction
        deployLeftConfig.ExternalFeedback.SensorToMechanismRatio = IntakeConstants.INTAKE_DEPLOY_GEAR_RATIO;

        // Set software limits
        deployLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        deployLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        deployLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.INTAKE_DEPLOYED_POSITION.in(Rotations);
        deployLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.INTAKE_START_POSITION.in(Rotations);

        // Set brake on boot and invert motor
        deployLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var deployStatus = this.intakeDeployLeftMotor.getConfigurator().apply(deployLeftConfig);
        SmartDashboard.putString("Intake/Deploy Config Status", deployStatus.toString());
        if (!deployStatus.isOK()) {
            DriverStation.reportError("Intake deploy configuration failed: " + deployStatus, false);
        }

        // Configure roller motors
        var rollerConfig = new TalonFXSConfiguration();

        // Roller motor: NEO Vortex with Solo Adapter and JST sensor connection.
        rollerConfig.Commutation.MotorArrangement = MotorArrangementValue.VORTEX_JST;

        // Set current limits
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_ROLLERS_SUPPLY_CURRENT_LIMIT;
        rollerConfig.Voltage.PeakForwardVoltage = IntakeConstants.INTAKE_ROLLER_MAX_VOLTAGE;
        rollerConfig.Voltage.PeakReverseVoltage = -IntakeConstants.INTAKE_ROLLER_MAX_VOLTAGE;

        this.intakeRollerMotor.getConfigurator().apply(rollerConfig);

        // Relative encoder only: the arm MUST be physically fully up when code starts.
        var zeroStatus = this.intakeDeployLeftMotor.setPosition(IntakeConstants.INTAKE_START_POSITION);
        SmartDashboard.putString("Intake/Deploy Zero Status", zeroStatus.toString());
        if (!zeroStatus.isOK()) {
            DriverStation.reportError("Intake deploy encoder zero failed: " + zeroStatus, false);
        }
    }

    public void runIntake(double speed) {
        // Power intake motors
        // speed: [-1.0, 1.0] for reverse to forward
        // Full input is now 6 V (half of nominal 12 V), in either direction.

        intakeRollerMotor.setControl(new VoltageOut(
                MathUtil.clamp(speed, -1.0, 1.0) * IntakeConstants.INTAKE_ROLLER_MAX_VOLTAGE));
    }

    public void setIntakePosition(boolean up) {
        // Set intake positions using Motion Magic - smooth, profiled position control
        // up: true = retracted (stowed), false = deployed (down to collect game pieces)
        // MotionMagicVoltage tells the motor controller to go to an exact position
        // following the trapezoidal/S-curve profile configured in the constructor

        var targetPosition = up
                ? IntakeConstants.INTAKE_START_POSITION
                : IntakeConstants.INTAKE_DEPLOYED_POSITION;

        setDeployTarget(targetPosition);
    }

    public void setDeployTarget(Angle position) {
        if (!Double.isFinite(position.in(Rotations))) {
            return;
        }
        this.targetPosition = Rotations.of(MathUtil.clamp(position.in(Rotations),
                IntakeConstants.INTAKE_START_POSITION.in(Rotations),
                IntakeConstants.INTAKE_DEPLOYED_POSITION.in(Rotations)));
        holdDeployPosition();
    }

    public void holdDeployPosition() {
        this.intakeDeployLeftMotor.setControl(new MotionMagicVoltage(this.targetPosition));
    }

    public boolean isAtPosition() {
        var currentPosition = getCurrentPosition();
        return currentPosition.minus(this.targetPosition).abs(Rotations)
                < IntakeConstants.INTAKE_POSITION_TOLERANCE.in(Rotations);
    }

    public Angle getCurrentPosition() {
        return this.intakeDeployLeftMotor.getPosition().getValue();
    }

    public Angle getTargetPosition() {
        return this.targetPosition;
    }

    public void jogPosition(Angle steps) {
        setDeployTarget(this.targetPosition.plus(steps));
    }

    @Override
    public void periodic() {
        publishTelemetry();
    }

    public void publishTelemetry() {
        SmartDashboard.putNumber("Intake/Current Position (tr)", intakeDeployLeftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Target Position (tr)", targetPosition.in(Rotations));
        SmartDashboard.putNumber("Intake/Current Position (deg)", getCurrentPosition().in(Degrees));
        SmartDashboard.putNumber("Intake/Target Position (deg)", targetPosition.in(Degrees));
        SmartDashboard.putNumber("Intake/Deploy Motor Voltage (V)", intakeDeployLeftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Deploy Stator Current (A)", intakeDeployLeftMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Test Enabled", DriverStation.isTestEnabled());
    }
}
