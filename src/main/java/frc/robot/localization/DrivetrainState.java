package frc.robot.localization;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.SwerveArcOdometry;

/** Maintains the drivetrain pose estimate using wheel odometry and vision observations. */
public class DrivetrainState {

    private boolean initted;
    private final PoseEstimator<SwerveModulePosition[]> visionAdjustedOdometry;
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

    /** Creates the drivetrain state estimator from the initial module positions and gyro yaw. */
    public DrivetrainState(SwerveModulePosition[] wheelPositions, Rotation2d gyroYaw) {
        SwerveDriveOdometry swerveOdometry =
            new SwerveArcOdometry(Constants.Swerve.swerveKinematics, gyroYaw, wheelPositions);
        visionAdjustedOdometry = new PoseEstimator<>(Constants.Swerve.swerveKinematics,
            swerveOdometry, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9));
    }

    /** Resets the estimated pose to a known field pose. */
    public void resetPose(Pose2d pose) {
        visionAdjustedOdometry.resetPose(pose);
        initted = true;
    }

    /** Allows the next accepted vision observation to initialize the pose again. */
    public void resetInit() {
        initted = false;
    }

    /** Updates odometry from a timestamped swerve measurement. */
    public void addOdometryObservation(SwerveModulePosition[] wheelPositions,
        Rotation2d gyroYaw, double timestamp) {
        visionAdjustedOdometry.updateWithTime(timestamp, gyroYaw, wheelPositions);
    }

    /** Updates the field-relative drivetrain speeds used when weighting vision observations. */
    public void updateMeasuredSpeeds(ChassisSpeeds robotRelativeSpeeds) {
        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,
            getGlobalPoseEstimate().getRotation());
        Logger.recordOutput("State/currentSpeeds", fieldRelativeSpeeds);
    }

    /** Initializes or corrects the pose estimate with a processed vision observation. */
    public void addVisionObservation(VisionObservation observation) {
        Pose2d robotPose =
            observation.cameraPose().plus(observation.robotToCamera().inverse()).toPose2d();
        if (!initted) {
            visionAdjustedOdometry.resetPose(robotPose);
            initted = true;
            Logger.recordOutput("State/VisionRobotPose", robotPose);
            return;
        }

        Pose2d before = visionAdjustedOdometry.getEstimatedPosition();
        visionAdjustedOdometry.addVisionMeasurement(robotPose, observation.timestamp(),
            observation.getStdDev());
        Pose2d after = visionAdjustedOdometry.getEstimatedPosition();
        Logger.recordOutput("State/Correction",
            after.getTranslation().getDistance(before.getTranslation()));
        Logger.recordOutput("State/VisionRobotPose", robotPose);
    }

    /** Returns the current estimated robot pose in field coordinates. */
    public Pose2d getGlobalPoseEstimate() {
        return visionAdjustedOdometry.getEstimatedPosition();
    }

    /** Returns the latest measured chassis speeds in the field frame. */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return fieldRelativeSpeeds;
    }
}
