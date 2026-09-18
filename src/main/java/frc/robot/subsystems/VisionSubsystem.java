package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class VisionSubsystem extends SubsystemBase {

    public record VisionTarget (double overheadAngle, double range) {}

    CommandSwerveDrivetrain drivetrain = null;

    PoseEstimate poseEstimate;
    boolean hasValidPose = false;
    boolean hasTarget = false;
    boolean hasShooterTarget = false;
    double tx = 0.0;
    double ty = 0.0;
    double ta = 0.0;
    Distance range = Inches.of(0.0);
    double tagId = -1;

    double shooterTX = 0.0;
    double shooterTY = 0.0;
    int shooterTagId = -1;

    private final AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        // Load the 2026 field layout from WPILib's built in resource
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    }

    @Override
    public void periodic() {
        // Read raw Limelight data every cycle
        this.hasTarget = LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
        this.tx = LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
        this.ty = LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
        this.ta = LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
        this.tagId = LimelightHelpers.getFiducialID(VisionConstants.LIMELIGHT_NAME);

        // Compute range using per-tag height from field layout
        if (this.hasTarget && this.tagId >= 0) {
            var tagHeight = getTagHeightInches((int)this.tagId);
            var heightToCover = tagHeight.minus(ShooterConstants.FORWARD_CAMERA_HEIGHT);
            var angleToCover = ShooterConstants.FORWARD_CAMERA_ANGLE.plus(Degrees.of(this.ty));
            var distanceToTag = heightToCover.div(Math.tan(degreesToRadians(angleToCover).magnitude()));

            this.range = distanceToTag.plus(ShooterConstants.FORWARD_CAMERA_SHOOTER_OFFSET);

            // Debug: intermediate distance-calc values
            SmartDashboard.putNumber("Vision/TagHeight", tagHeight.magnitude());
            SmartDashboard.putNumber("Vision/HeightToCover", heightToCover.magnitude());
            SmartDashboard.putNumber("Vision/AngleToCover", angleToCover.magnitude());
            SmartDashboard.putNumber("Vision/RawDistToTag", distanceToTag.magnitude());
        } else {
            this.range = Inches.of(0.0);
        }

        // --- MegaTag2 Pose Estimation Pipeline ---
        if (drivetrain != null) {
            // Feed Pigeon yaw to Limelight for MegaTag2
            var yawDegrees = drivetrain.getPigeon2().getYaw().getValue();
            LimelightHelpers.SetRobotOrientation(VisionConstants.LIMELIGHT_NAME, yawDegrees.magnitude(),
                    0.0, 0.0, 0.0, 0.0, 0.0);

            // Read MegaTag2 Pose Estimates
            this.poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);
            this.hasValidPose = LimelightHelpers.validPoseEstimate(this.poseEstimate) && this.poseEstimate.tagCount > 0;

            // Feed valid vision pose into drivetrain Kalman filter
            if (this.hasValidPose) {
                // Scale standard deviations by tag count and distance
                // More tags + closer = lower stdDev = more trust
                double xyStdDev = 0.7; // base standard deviation in meters
                double rotStdDev = 9999999.0; // MegaTag2 handles rotation via IMU, don't trust vision rotation

                if (this.poseEstimate.tagCount >= 2) {
                    xyStdDev = 0.5;
                }

                if (this.poseEstimate.avgTagDist > 4.0) {
                    // Reduce trust for distant tags (>4 meters)
                    xyStdDev *= (this.poseEstimate.avgTagDist / 4.0);
                }

                this.drivetrain.addVisionMeasurement(
                        this.poseEstimate.pose,
                        this.poseEstimate.timestampSeconds,
                        VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
                );
            }
        }

        // Scan all visible fiducials for the specific shooter tag (9 or 25)
        this.hasShooterTarget = false;
        var rawFiducials = LimelightHelpers.getRawFiducials(VisionConstants.LIMELIGHT_NAME);
        for (var fid : rawFiducials) {
            if (fid.id == AprilTagConstants.RED_SHOOTER_TAG_ID || fid.id == AprilTagConstants.BLUE_SHOOTER_TAG_ID) {
                this.shooterTX = fid.txnc;
                this.shooterTY = fid.tync;
                this.shooterTagId = fid.id;
                this.hasShooterTarget = true;
                break;
            }
        }

        if (!this.hasShooterTarget) {
            this.shooterTX = 0.0;
            this.shooterTY = 0.0;
            this.shooterTagId = -1;
        }

        // Publish to smart dashboard
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/TX", tx);
        SmartDashboard.putNumber("Vision/TY", ty);
        SmartDashboard.putNumber("Vision/TA", ta);
        SmartDashboard.putNumber("Vision/Range", range.magnitude());
        SmartDashboard.putNumber("Vision/TagID", tagId);
        SmartDashboard.putBoolean("Vision/IsShootableTag", hasValidShooterTarget());
        SmartDashboard.putBoolean("Vision/HasValidPose", hasValidPose);
        if (hasValidPose) {
            SmartDashboard.putNumber("Vision/PoseX", poseEstimate.pose.getX());
            SmartDashboard.putNumber("Vision/PoseY", poseEstimate.pose.getY());
            SmartDashboard.putNumber("Vision/PoseRot", poseEstimate.pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/TagCount", poseEstimate.tagCount);
            SmartDashboard.putNumber("Vision/AvgTagDist", poseEstimate.avgTagDist);
        }
    }

    public VisionTarget getVisionTarget() {
        if (this.hasShooterTarget) {
            // Use shooter-specific tag data from raw fiducial scan
            var tagHeight = getTagHeightInches(this.shooterTagId);
            var heightToCover = tagHeight.minus(ShooterConstants.FORWARD_CAMERA_HEIGHT);
            var angleToCover = ShooterConstants.FORWARD_CAMERA_ANGLE.plus(Degrees.of(this.ty));
            var distanceToTag = heightToCover.div(Math.tan(degreesToRadians(angleToCover).magnitude()));

            // Calculate distance from shooter to hub center
            var xOffsetFromHub = distanceToTag.magnitude() * Math.cos(degreesToRadians(this.shooterTX));
            var yOffsetFromHub = distanceToTag.magnitude() * Math.sin(degreesToRadians(this.shooterTX));
            var xOffsetFromTarget = xOffsetFromHub + ShooterConstants.HUB_WIDTH_MAX.magnitude() / 2.0;

            double overheadAngleTarget = Math.atan(xOffsetFromTarget / yOffsetFromHub);
            double range = Math.sqrt(xOffsetFromTarget * xOffsetFromTarget + yOffsetFromHub * yOffsetFromHub);
            range += ShooterConstants.FORWARD_CAMERA_SHOOTER_OFFSET.magnitude();

            return new VisionTarget(overheadAngleTarget, range);
        } else {
            // If shooter AprilTag not visible, shoot straight ahead, assuming robot is at base position
            var angle = 0.0;
            var range = ShooterConstants.BASE_RANGE.minus(ShooterConstants.SHOOTER_OFFSET_FROM_REAR_BUMPER);
            return new VisionTarget (angle, range.magnitude());
        }
    }

    public void setVisionTarget(VisionTarget target) {
        target = getVisionTarget();
    }

    public Optional<Pose3d> getTagFieldPose(int tagId) {
        return this.fieldLayout.getTagPose(tagId);
    }

    public Distance getTagHeightInches(int tagId) {
        var pose = this.fieldLayout.getTagPose(tagId);
        // Field layout Z is in meters, convert to inches
        return pose.map(pose3d -> Inches.of(pose3d.getZ())).orElse(ShooterConstants.HUB_TAG_HEIGHT);

        // Fallback to hardcoded hub tag height if tag not found in layout
    }

    public AprilTagFieldLayout getFieldLayout() {
        return this.fieldLayout;
    }

    public PoseEstimate getLatestPoseEstimate() {
        return this.poseEstimate;
    }

    public boolean hasValidPoseEstimate() {
        return this.hasValidPose;
    }

    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public boolean hasValidShooterTarget() {
        return this.hasShooterTarget;
    }

    public double getTX() {
        return this.shooterTX;
    }

    private Angle degreesToRadians(Angle degrees) {
        return degrees.times(0.0174532925);
    }

    private double degreesToRadians(double degrees) {
        return degrees * 0.0174532925;
    }
}
