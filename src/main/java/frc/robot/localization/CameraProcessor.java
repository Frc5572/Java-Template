package frc.robot.localization;

import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.vision.CameraConstants;

/**
 * Filters a {@link org.photonvision.targeting.PhotonPipelineResult} and, when the result passes all
 * quality checks, produces a {@link VisionObservation} ready for fusion into
 * {@link DrivetrainState}.
 *
 * <p>
 * This class is stateless with respect to the robot pose it only decides whether a camera frame is
 * trustworthy and computes the appropriate standard deviations. Pose-estimator mutations remain the
 * exclusive responsibility of {@link DrivetrainState}.
 *
 */
public class CameraProcessor {

    private final CameraConstants cameraConstants;

    /** Creates a camera processor. */
    public CameraProcessor(CameraConstants cameraConstants) {
        this.cameraConstants = cameraConstants;
    }

    /**
     * processes photon results to get robots pose
     *
     * @param result photon results
     * @param currentSpeeds robot speeds
     * @return position data
     */
    public Result<VisionObservation, RejectionReason> process(PhotonPipelineResult result,
        ChassisSpeeds currentSpeeds) {
        var multiTag = result.getMultiTagResult();
        Transform3d robotToCamera = cameraConstants.robotToCamera;

        double translationSpeed =
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double rotationSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);

        double velocityStdDev = cameraConstants.latencyStdDev.in(Seconds);
        double velocityTranslationError = translationSpeed * velocityStdDev;
        double velocityRotationError = rotationSpeed * velocityStdDev;
        Logger.recordOutput("State/velocityTranslationError", velocityTranslationError);
        Logger.recordOutput("State/velocityRotationError", velocityRotationError);

        var bestTarget = result.hasTargets() ? result.getBestTarget() : null;
        if (bestTarget == null) {
            return Result.err(RejectionReason.NO_TARGETS);
        }

        var bestTagPose = Constants.Vision.fieldLayout.getTagPose(bestTarget.getFiducialId());
        if (bestTagPose.isEmpty()) {
            return Result.err(RejectionReason.NO_TARGETS);
        }

        if (multiTag.isPresent()) {
            Transform3d best = multiTag.get().estimatedPose.best;
            Pose3d cameraPose =
                new Pose3d().plus(best).relativeTo(Constants.Vision.fieldLayout.getOrigin());

            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/cameraPose", cameraPose);
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/correctedCameraPose",
                cameraPose);

            Pose3d estRobotPose = cameraPose.plus(robotToCamera.inverse());
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/estRobotPose",
                estRobotPose);

            double stdDevMultiplier = stdDevMultiplier(result.targets, cameraPose);
            double translationStdDev =
                stdDevMultiplier * cameraConstants.translationError + velocityTranslationError;
            double rotationStdDev =
                stdDevMultiplier * cameraConstants.rotationError + velocityRotationError;

            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevMultipler",
                stdDevMultiplier);
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevTranslation",
                translationStdDev);
            Logger.recordOutput("State/Camera/" + cameraConstants.name + "/stdDevRotation",
                rotationStdDev);

            return Result.ok(new VisionObservation(cameraPose, robotToCamera, translationStdDev,
                rotationStdDev, result.getTimestampSeconds()));
        }

        return Result.err(RejectionReason.SINGLE_TAG_ONLY);
    }


    private static double stdDevMultiplier(List<PhotonTrackedTarget> targets, Pose3d cameraPose) {
        double totalDistance = 0.0;
        int count = 0;
        for (var tag : targets) {
            var maybeTagPose = Constants.Vision.fieldLayout.getTagPose(tag.getFiducialId());
            if (maybeTagPose.isPresent()) {
                var tagPose = maybeTagPose.get();
                totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
                count++;
            }
        }
        double avgDistance = totalDistance / count;
        double stddev = Math.pow(avgDistance, 2.0) / count;
        return stddev;
    }

    /** results tuple */
    public sealed interface Result<T, E> permits Ok, Err {
        public static <T, E> Result<T, E> ok(T value) {
            return new Ok<>(value);
        }

        public static <T, E> Result<T, E> err(E error) {
            return new Err<>(error);
        }
    }

    /** Ok results */
    public record Ok<T, E>(T value) implements Result<T, E> {
    }

    /** Err results */
    public record Err<T, E>(E error) implements Result<T, E> {
    }
}
