package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    VictorSP intakeMotor1 = new VictorSP(1);
    VictorSP intakeMotor2 = new VictorSP(2);

    public DigitalInput touchSensor = new DigitalInput(1);

    public Intake() {
        intakeMotor2.setInverted(true);
        intakeMotor1.addFollower(intakeMotor2);
    }

    public void setPower(double power) {
        intakeMotor1.set(power);
    }

    public void intake() {
        setPower(-0.7);
    }

    public void outtake() {
        setPower(0.7);
    }

    public void stop() {
        setPower(0);
    }

    public Command intakeCommand() {
        return Commands.runEnd(this::intake, this::stop, this); // .until(() -> beambrake.get());
    }

}
