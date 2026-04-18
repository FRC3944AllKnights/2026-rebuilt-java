package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static com.ctre.phoenix6.HootSchemaType.Struct;

public class Telemetry {
    private LinearVelocity maxSpeed;

    /* What to publish over networktables for telemetry */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    NetworkTable driveStateTable = inst.getTable("DriveState");
    StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    NetworkTable table = inst.getTable("Pose");
    DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    Mechanism2d[] moduleMechanisms = {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1)
    };

    /* A direction and length changing ligament for speed representation */
    MechanismLigament2d[] moduleSpeeds = {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, .5).append(new MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, .5).append(new MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, .5).append(new MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, .5).append(new MechanismLigament2d("Speed", 0.5, 0.0))
    };

    /* A direction changing and length constant ligament for module direction */
    MechanismLigament2d[] moduleDirections = {
            moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0.0, 0.0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0.0, 0.0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0.0, 0.0, new Color8Bit(Color.kWhite))),
            moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                    .append(new MechanismLigament2d("Direction", 0.1, 0.0, 0.0, new Color8Bit(Color.kWhite)))
    };

    public Telemetry(LinearVelocity maxSpeed) {
        this.maxSpeed = maxSpeed;

        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < moduleSpeeds.length; i++) {
            SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
        }
    }

    public void Telemeterize(SwerveDrivetrain.SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod);

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
                state.Pose.getX(),
                state.Pose.getY(),
                state.Pose.getRotation().getDegrees()
        });

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < moduleSpeeds.length; ++i) {
            moduleDirections[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle.getDegrees());
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2.0 * maxSpeed.magnitude()));
        }
    }
}
