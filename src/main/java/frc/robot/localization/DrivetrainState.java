package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DrivetrainState {

    public void addOdometryObservation(OdometryObservation observation);

    public void addVisionObservation(VisionObservation observation);

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds);

    public Pose2d getPose();

    public ChassisSpeeds getRobotRelativeSpeeds();

    public default ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(),
            getPose().getRotation());
    }

}
