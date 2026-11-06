package frc.robot;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.jspecify.annotations.NullMarked;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOEmpty;
import frc.robot.subsystems.swerve.SwerveReal;
import frc.robot.subsystems.swerve.gyro.GyroIOEmpty;
import frc.robot.subsystems.swerve.gyro.GyroPigeon2;
import frc.robot.subsystems.swerve.mod.SwerveModuleIOEmpty;
import frc.robot.subsystems.swerve.mod.SwerveModuleReal;
import frc.robot.subsystems.swerve.util.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.util.TeleopControls;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOEmpty;
import frc.robot.subsystems.vision.VisionReal;
import frc.robot.util.DeviceDebug;
import frc.robot.viz.RobotViz;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@NullMarked
public final class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver =
        new CommandXboxController(Constants.DriverControls.controllerId);

    /* Subsystems */
    private final Swerve swerve;
    private final Vision vision;
    private final RobotViz viz;
    private final Lock odometryLock = new ReentrantLock();
    private final PhoenixOdometryThread odometryThread;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        CANBus canBus = new CANBus(Constants.Swerve.isCanviore ? "*" : "rio");
        odometryThread = new PhoenixOdometryThread(odometryLock, canBus);
        Swerve.Bundle swerveBundle;

        switch (runtimeType) {
            case kReal:
                swerveBundle = Swerve.create(odometryLock, odometryThread, SwerveReal::new,
                    GyroPigeon2::new, SwerveModuleReal::new);
                vision = new Vision(swerveBundle.state(), new VisionReal());
                break;
            default:
                swerveBundle = Swerve.create(odometryLock, odometryThread, SwerveIOEmpty::new,
                    GyroIOEmpty::new, SwerveModuleIOEmpty::new);
                vision = new Vision(swerveBundle.state(), new VisionIOEmpty());
        }
        swerve = swerveBundle.swerve();
        viz = new RobotViz(swerve);

        odometryThread.start();

        DeviceDebug.initialize();

        swerve.setDefaultCommand(swerve.driveUserRelative(TeleopControls.teleopControls(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX())));

        driver.y().onTrue(swerve.setFieldRelativeOffset());

        driver.a().whileTrue(swerve.wheelRadiusCharacterization()).onFalse(swerve.emergencyStop());
        driver.b().whileTrue(swerve.feedforwardCharacterization()).onFalse(swerve.emergencyStop());
    }

    /** Runs once per 0.02 seconds after subsystems and commands. */
    public void periodic() {
        viz.periodic();
    }
}
