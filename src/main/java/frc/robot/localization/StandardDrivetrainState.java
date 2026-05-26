package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class StandardDrivetrainState implements DrivetrainState {

    @Override
    public void addOdometryObservation(OdometryObservation observation) {

        throw new UnsupportedOperationException("Unimplemented method 'addOdometryObservation'");
    }

    @Override
    public void addVisionObservation(VisionObservation observation) {

        throw new UnsupportedOperationException("Unimplemented method 'addVisionObservation'");
    }

    @Override
    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {

        throw new UnsupportedOperationException("Unimplemented method 'setRobotRelativeSpeeds'");
    }

    @Override
    public Pose2d getPose() {

        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {

        throw new UnsupportedOperationException("Unimplemented method 'getRobotRelativeSpeeds'");
    }



}
