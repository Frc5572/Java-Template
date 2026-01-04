package frc.robot.subsystems.swerve.gyro;

import java.util.Queue;
import org.jspecify.annotations.NullMarked;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;

/** NavX2 implementation for Gyro */
@NullMarked
public class GyroNavX2 implements GyroIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID, (int) Constants.Swerve.odometryFrequency);

    private final Queue<Double> yawQueue;

    /** NavX2 implementation for Gyro */
    public GyroNavX2(PhoenixOdometryThread odometryThread) {
        this.yawQueue = odometryThread.registerSignal(() -> Units.degreesToRadians(gyro.getYaw()));
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = gyro.isConnected();

        inputs.yaw = Rotation2d.fromDegrees(gyro.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRawGyroZ());

        inputs.pitch = Rotation2d.fromDegrees(gyro.getPitch());
        inputs.pitchVelocityRadPerSec = Units.degreesToRadians(gyro.getRawGyroX());

        inputs.roll = Rotation2d.fromDegrees(gyro.getRoll());
        inputs.rollVelocityRadPerSec = Units.degreesToRadians(gyro.getRawGyroY());

        inputs.yawRads = yawQueue.stream().mapToDouble(x -> x).toArray();
        yawQueue.clear();
    }

}
