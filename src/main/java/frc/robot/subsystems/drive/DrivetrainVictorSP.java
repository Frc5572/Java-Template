package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * Drivetrain VictorSP
 */
public class DrivetrainVictorSP implements DrivetrainIO {
    //create all 4 directions for both motars

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    /**
     * Drivetrain VictorSP
     */
    public DrivetrainVictorSP() {
        //make the motars follow each other and then make them inverted
        
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        inputs.gyroYaw = Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Drive Voltage
     */
    public void setDriveVoltage(double lvolts, double rvolts) {
        //set the volts to the motar
        
    }

}
