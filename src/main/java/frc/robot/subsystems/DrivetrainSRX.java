// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DrivetrainSRX extends SubsystemBase {
  public final static double MAX_VELOCITY_METERS_PER_SECOND = .1;
  TalonSRX talonDriveRight = new TalonSRX(Robot.config.driveRight);
  TalonSRX talonDriveLeft = new TalonSRX(Robot.config.driveLeft);
  XboxController driveController;
  private static SlewRateLimiter sLX = new SlewRateLimiter(DrivetrainSRX.MAX_VELOCITY_METERS_PER_SECOND);
  private static SlewRateLimiter sLY = new SlewRateLimiter(DrivetrainSRX.MAX_VELOCITY_METERS_PER_SECOND);
  private Double targetAngle = null;

  public DrivetrainSRX(XboxController driveController) {
    
    logf("Start of Drive Train for SRX Subsystem\n");
    this.driveController = driveController;
    talonDriveRight.configFactoryDefault();
    talonDriveRight.setInverted(false); // pick CW versus CCW when motor controller is positive/green
    talonDriveLeft.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Make sure that you declare this subsystem in RobotContainer.java
    double leftStick = -driveController.getLeftY();
    sLX.calculate(leftStick);
    talonDriveLeft.set(ControlMode.PercentOutput, leftStick); 
    sLY.calculate(leftStick);
    double rightStick = driveController.getRightY(); // make forward stick positive
    talonDriveRight.set(ControlMode.PercentOutput, rightStick);
    if (Robot.count % 250 == -1) { // -1 will disable the log, set to 0 to enable log
      logf("Drive stick left:%.2f right:%.2f\n", leftStick, rightStick);
    }
    SmartDashboard.putNumber("Right Stick", rightStick);
    SmartDashboard.putNumber("Left Stick", leftStick);
  }

  void driveStraight(double rightJoy, double leftJoy) {
    double error = Robot.yaw - targetAngle;
    if (error > 10) {
        logf("!!!!! Drive Straight error too positive diff:%.1f yaw:%.1f target:%.3f\n", error,
                Robot.yaw, targetAngle);
        error = 5;
    }
    if (error < -10) {
        logf("!!!!! Drive Straight error too negative diff:%.1f yaw:%.1f target:%.3f\n", error,
                Robot.yaw, targetAngle);
        error = -5;
    }
    // Adjsut speed if too fast
    double averageJoy = (rightJoy + leftJoy) / 1.0;
    // If turbo mode ignore speed limit
    // if (!turboMode) {
    // if (averageJoy > .6)
    // averageJoy = .6;
    // if (averageJoy < -.6)
    // averageJoy = -.6;
    // }
    double factor = error * Math.abs(averageJoy) * 0.035; // Was 0.045
    // Log drive straight data every 2.5 seconds
    if (Robot.count % 12 == 0) {
        logf("Drive Straight targ:%.2f yaw:%.2f err:%.2f avg:%.2f factor:%.2f Joy:<%.2f,%.2f>\n",
                targetAngle, Robot.yaw, error,
                averageJoy, factor,  rightJoy, leftJoy);
    }
    leftJoy = averageJoy - factor;
    rightJoy = averageJoy + factor;
}

}