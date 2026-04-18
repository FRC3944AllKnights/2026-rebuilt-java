package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.LEDStrip;
import frc.robot.Telemetry;

import static edu.wpi.first.units.Units.*;

public class ShooterSubsystem extends SubsystemBase {

    VisionSubsystem vision = null;
    boolean adjustableRPM = false;

    TalonFXS shooterLeftMotor = new TalonFXS(CANConstants.SHOOTER_LEFT_MOTOR_ID, CANConstants.CAN_BUS);
    TalonFXS shooterRightMotor = new TalonFXS(CANConstants.SHOOTER_RIGHT_MOTOR_ID, CANConstants.CAN_BUS);
    TalonFXS indexerLeftMotor = new TalonFXS(CANConstants.INDEXER_LEFT_MOTOR_ID, CANConstants.CAN_BUS);
    TalonFXS indexerRightMotor = new TalonFXS(CANConstants.INDEXER_RIGHT_MOTOR_ID, CANConstants.CAN_BUS);
    LEDStrip ledStrip = new LEDStrip(ShooterConstants.LED_PORT, ShooterConstants.NUMBER_OF_LEDS);

    public ShooterSubsystem() {
        // Configure shooter motors
        // Left motor - set up as lead motor
        var shooterLeftMotorConfig = new TalonFXSConfiguration();

        // Set motor to NEO
        shooterLeftMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

        // Set PID values
        shooterLeftMotorConfig.Slot0.withKP(ShooterConstants.SHOOTER_P);
        shooterLeftMotorConfig.Slot0.withKI(ShooterConstants.SHOOTER_I);
        shooterLeftMotorConfig.Slot0.withKD(ShooterConstants.SHOOTER_D);
        shooterLeftMotorConfig.Slot0.withKS(ShooterConstants.SHOOTER_S);
        shooterLeftMotorConfig.Slot0.withKV(ShooterConstants.SHOOTER_V);

        // Set current limits
        shooterLeftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterLeftMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT;

        // Apply configuration
        this.shooterLeftMotor.getConfigurator().apply(shooterLeftMotorConfig);

        // Right motor - set up as a follower of the left motor
        var shooterRightMotorConfig = new TalonFXSConfiguration();

        // Set motor to NEO
        shooterRightMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

        // Set current limits
        shooterRightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterRightMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOTER_SUPPLY_CURRENT_LIMIT;

        // Apply configuration
        this.shooterRightMotor.getConfigurator().apply(shooterRightMotorConfig);

        // Set as follower
        Follower rightMotorControlMethod = new Follower(CANConstants.SHOOTER_LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);
        this.shooterRightMotor.setControl(rightMotorControlMethod);

        // Set up indexer motors
        // Left motor - set up as lead motor
        var indexerLeftMotorConfig = new TalonFXSConfiguration();

        // Set up motor to NEO
        indexerLeftMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

        // Set up PID values
        indexerLeftMotorConfig.Slot0.withKP(ShooterConstants.INDEXER_P);
        indexerLeftMotorConfig.Slot0.withKI(ShooterConstants.INDEXER_I);
        indexerLeftMotorConfig.Slot0.withKD(ShooterConstants.INDEXER_D);

        // Set current limits
        indexerLeftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerLeftMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.INDEXER_SUPPLY_CURRENT_LIMIT;

        // Apply configuration
        this.indexerLeftMotor.getConfigurator().apply(indexerLeftMotorConfig);

        // Right motor - set up as follower of the left motor
        var indexerRightMotorConfig = new TalonFXSConfiguration();

        // Set up motor as NEO
        indexerRightMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO_JST;

        // Set up current limits
        indexerRightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerRightMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.INDEXER_SUPPLY_CURRENT_LIMIT;

        // Apply configuration
        this.indexerRightMotor.getConfigurator().apply(indexerRightMotorConfig);

        // Set as follower
        Follower rightIndexerMotorControlMethod = new Follower(CANConstants.INDEXER_LEFT_MOTOR_ID, MotorAlignmentValue.Opposed);
        this.indexerRightMotor.setControl(rightIndexerMotorControlMethod);

        // LED Strip
        // this.LEDStrip = LEDStrip(ShooterConstants.LEDPort, ShooterConstants.NumberOfLEDs);
        /*
        this.led.setLength(ShooterConstants.NumberOfLEDs);
        this.led.setData(this.ledBuffer);
        this.led.start();
        */
    }

    @Override
    public void periodic() {
        // Passive debug telemetry - publish computed RPM without requiring trigger input
        SmartDashboard.putBoolean("Shooter/AdjustableRPMEnabled", this.adjustableRPM);

        if (vision != null) {
            var isShootable = vision.hasValidShooterTarget();

            if (this.adjustableRPM && isShootable) {
                var target = vision.getVisionTarget();
                var computedRPM = getTargetShooterRPM(target.range());
                SmartDashboard.putNumber("Shooter/ComputedRPM", computedRPM);
            } else {
                SmartDashboard.putNumber("Shooter/ComputedRPM", 0.0);
            }
        }
    }

    public void spinUpShooter(double speed) {
        // Set shooter motors to speed
        // speed: speed from 0 to 1.0

        // Currently for prototyping only

        Angle baseRPM = Revolutions.of(2650.00);
        double gearRatio = 1.0;
        Angle targetRPM;

        if (this.adjustableRPM && this.vision != null && speed > 0.0) {
            if (vision.hasValidShooterTarget()) {
                var target = this.vision.getVisionTarget();
                double computedRPM = getTargetShooterRPM(target.range());
                targetRPM = Revolutions.of(computedRPM);
            } else {
                targetRPM = Revolutions.of(baseRPM.magnitude() / gearRatio);
            }
        } else {
            targetRPM = Revolutions.of(speed * baseRPM.magnitude() / gearRatio);
        }

        AngularVelocity targetTPS = RevolutionsPerSecond.of(targetRPM.magnitude() / 60.0);

        if (speed < 0.01) {
            this.shooterLeftMotor.setControl(new NeutralOut());
        } else {
            VelocityVoltage request = new VelocityVoltage(targetTPS);
            this.shooterLeftMotor.setControl(request.withVelocity(targetTPS).withSlot(0));
        }

        // Output to dashboard for testing

        SmartDashboard.putNumber("Shooter Wheel Target RPM", targetRPM.magnitude());
        var actualRPM = shooterLeftMotor.getRotorVelocity().getValue().times(60.0);
        SmartDashboard.putNumber("Shooter Wheel Actual RPM", actualRPM.magnitude());

        if (ShooterConstants.DEBUG_PRINTS_ENABLED) {
            SmartDashboard.putNumber("Speed Commanded [0, 1]", speed);

            var leftSupplyVoltage = this.shooterLeftMotor.getSupplyVoltage().getValueAsDouble();
            var rightSupplyVoltage = this.shooterRightMotor.getSupplyVoltage().getValueAsDouble();
            SmartDashboard.putNumber("Shooter Left Supply Voltage", leftSupplyVoltage);
            SmartDashboard.putNumber("Shooter Right Supply Voltage", rightSupplyVoltage);

            var rightMotorRPM = shooterRightMotor.getRotorVelocity().getValue().times(60.0);
            var percentDifferenceShootersMotor = 100.0 * (actualRPM.plus(rightMotorRPM).magnitude()) / ((actualRPM.plus(rightMotorRPM)).magnitude() / 2.0);
            SmartDashboard.putNumber("Shooter Motors RPM Percent Difference", percentDifferenceShootersMotor);

            var leftMotorPosition = shooterLeftMotor.getRotorPosition().getValueAsDouble();
            var rightMotorPosition = shooterRightMotor.getRotorPosition().getValueAsDouble();
            double percentDifferenceShooterMotorsTheta = 100.0 * (leftMotorPosition - rightMotorPosition) / ((leftMotorPosition - rightMotorPosition) / 2.0);
            SmartDashboard.putNumber("Shooter Motors Theta Percent Difference", percentDifferenceShooterMotorsTheta);
        }

        // Set LED if shooter is commanded
        /*
        if (speed > 0.0) {
            this.LEDStrip.setLEDs('Y'); // Set to yellow when shooter is commanded
        } else {
            this.LEDStrip.setLEDs('B');
        }
         */
    }

    public void setIndexerSpeed(double speed) {
        // Set indexer motor speed
        // speed: speed from 0 to 1.0

        if (speed <= 0.01) {
            this.indexerLeftMotor.setControl(new NeutralOut());
        } else {
            this.indexerLeftMotor.setControl(new DutyCycleOut(speed));
        }

        if (ShooterConstants.DEBUG_PRINTS_ENABLED) {
            SmartDashboard.putNumber("Indexer Commanded Speed", speed);
            double actualRPM = indexerLeftMotor.getRotorVelocity().getValueAsDouble() * 60.0;
            SmartDashboard.putNumber("Indexer Actual RPM", actualRPM);
        }
    }

    public double getTargetShooterRPM(double rangeIn) {
        // Returns the desired RPM based on vision data
        // range: distance from hub in inches (measured from ball exit location to hub center)
        // if range < minimumRange: Calculated directly from vision data (including for negative inputs)
        // if AprilTag not visible: Calculate from base position

        Distance range;
        if (rangeIn < ShooterConstants.MINIMUM_RANGE.magnitude()) {
            range = ShooterConstants.MINIMUM_RANGE;
        } else {
            range = Inches.of(rangeIn);
        }

        double b;
        if (ShooterConstants.EXIT_ANGLE.equals(Degrees.of(45.0))) {
            b = 1.0;
        } else if (ShooterConstants.EXIT_ANGLE.equals(Degrees.of(60.0))) {
            b = 1.73205;
        } else {
            b = Math.tan(ShooterConstants.EXIT_ANGLE.magnitude() * Math.PI / 180.0);
        }

        double a = (ShooterConstants.HUB_HEIGHT.minus(ShooterConstants.SHOOTER_HEIGHT)).magnitude() - b * range.magnitude() / (Math.pow(range.magnitude(), 2.0));

        // Extract exit velocity directly from trajectory coefficient a.
        // a encodes -g / (2 * v^2 * cos^2(theta)), so solve for v:
        double cosTheta = Math.cos(ShooterConstants.EXIT_ANGLE.magnitude() * Math.PI / 180.0);
        double exitSpeed = Math.sqrt(-ShooterConstants.GRAVITATIONAL_ACCELERATION.magnitude()) / (2.0 * a * Math.pow(cosTheta, 2.0));
        double rpmNominal = (exitSpeed / ShooterConstants.SHOOTER_CIRCUMFERENCE.magnitude()) * 60.0;

        if (ShooterConstants.DEBUG_PRINTS_ENABLED) {
            SmartDashboard.putNumber("Shooter/Debug/InputRange", rangeIn);
            SmartDashboard.putNumber("Shooter/Debug/ClampedRange", range.magnitude());
            SmartDashboard.putNumber("Shooter/Debug/CoeffA", a);
            SmartDashboard.putNumber("Shooter/Debug/ExitSpeedInPerSec", exitSpeed);
            SmartDashboard.putNumber("Shooter/Debug/RPMNominal", rpmNominal);
        }

        return rpmNominal / ShooterConstants.SHOOTER_BALL_SPEED_TRANSFER_PERCENT;
    }

    public void setVision(VisionSubsystem vision) {
        this.vision = vision;
    }

    public void setAdjustableRPM(boolean enabled) {
        this.adjustableRPM = enabled;
    }

    public void toggleAdjustableRPM() {
        this.adjustableRPM = !this.adjustableRPM;
    }

    public boolean isAdjustableRPMEnabled() {
        return this.adjustableRPM;
    }

    private double degreesToRadians(Angle degrees) {
        return degrees.times(0.0174532925).magnitude();
    }

    private void setLEDs(char patternId) {
        this.ledStrip.setLEDs(patternId);
    }
}
