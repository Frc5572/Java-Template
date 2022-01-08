// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project

package frc.robot;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import static java.lang.Math.abs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class Robot extends TimedRobot {

  XboxController driver = new XboxController(0);
  Joystick driverJ = new Joystick(0);
  XboxController operator = new XboxController(1);

  VictorSP rDrive1 = new VictorSP(6);
  VictorSP rDrive2 = new VictorSP(7);
  SpeedControllerGroup rightDriveMotors = new SpeedControllerGroup(rDrive1, rDrive2);

  VictorSP lDrive1 = new VictorSP(4);
  VictorSP lDrive2 = new VictorSP(5);
  SpeedControllerGroup leftDriveMotors = new SpeedControllerGroup(lDrive1, lDrive2);

  VictorSP elevator = new VictorSP(3);

  VictorSP intake1 = new VictorSP(1);
  VictorSP intake2 = new VictorSP(2);
  SpeedControllerGroup intakeMotors = new SpeedControllerGroup(intake1, intake2);

  VictorSP climb = new VictorSP(0);

  DigitalInput climbSwitch = new DigitalInput(0);
  DigitalInput elevSwitch = new DigitalInput(1);

  Encoder encoder1 = new Encoder(3, 4, true);
  Encoder encoder2 = new Encoder(5, 6, false);

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
    encoder1.setDistancePerPulse(1. / 256.);
    encoder1.reset();
    rightDriveMotors.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    // System.out.println("Robot Periodic");
  }

  @Override
  public void autonomousInit() {
    encoder1.reset();
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println(encoder1.getDistance());
    // System.out.println(encoder2.getDistance());
    if (encoder1.getDistance() < 2) {
      leftDriveMotors.set(1);
      rightDriveMotors.set(1);
    } else {
      leftDriveMotors.set(0);
      rightDriveMotors.set(0);
    }
  }

  @Override
  public void teleopInit() {
    // System.out.println("Teleop Init");
  }

  @Override
  public void teleopPeriodic() {
    Drive();
    Take();
    Elevator();
    Climb();
    System.out.println(climbSwitch.get());
    System.out.println(elevSwitch.get());
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /* ---------------FUN--------------- */

  int MotorSpeed = 1;

  /*
  1 = Full Power
  2 = Half Power
  3 = 1/3 Power
  4 = 1/4 Power
  and so on
  */

  void Drive() {
    if (abs(driver.getRawAxis(1)) > .01) {
      leftDriveMotors.set(driver.getRawAxis(1) / MotorSpeed);
    } else {
      leftDriveMotors.set(0);
    }
    // new controllers have axis = 3, old controllers = 5
    if (abs(driver.getRawAxis(3)) > .01) {
      rightDriveMotors.set(driver.getRawAxis(3) / MotorSpeed);
    } else {
      rightDriveMotors.set(0);
    }
  }

  void Take() {
    if (operator.getXButton()) {
      intakeMotors.set(1);
    } else if (operator.getYButton()) {
      intakeMotors.set(-1);
    } else {
      intakeMotors.set(0);
    }
  }

  void Elevator() {
    if (operator.getBButton() && !elevSwitch.get()) {
      elevator.set(-.5);
    } else {
      elevator.set(0);
    }
  }

  void Climb() {
    if (driverJ.getPOV() == 0 && climbSwitch.get()) {
      climb.set(-.5);
    } else if (driverJ.getPOV() == 180) {
      climb.set(.5);
    } else {
      climb.set(0);
    }
  }
}
