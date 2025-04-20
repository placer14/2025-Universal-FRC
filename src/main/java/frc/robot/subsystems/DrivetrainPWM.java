package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DrivetrainPWM extends SubsystemBase {
  private static DifferentialDrive robotDrive;
  private PWMSparkMax motorRight = new PWMSparkMax(8);
  private PWMSparkMax motorLeft = new PWMSparkMax(9);
  private XboxController driveHID;

  public double speedRatio = 0.5;
  public boolean straightMode = false;
  public DriveMode mode = DriveMode.CurvatureDrive;
  public Double targetAngle = null;
  private double leftJoy;
  private double rightJoy;
  private double yaw;

  public enum DriveMode {
    TankDrive,
    ArcadeDrive,
    CurvatureDrive,
  }

  public DrivetrainPWM(XboxController driveHID) {
    this.driveHID = driveHID;
    motorRight.setInverted(true); // Invert right side motors due to the construction of the frame
    motorRight.setSafetyEnabled(true);
    motorLeft.setSafetyEnabled(true);
    robotDrive = new DifferentialDrive((speed) -> motorLeft.set(speed * speedRatio),
        (speed) -> motorRight.set(speed * speedRatio * 0.90));
  }

  @Override
  public void periodic() {
    switch (mode) {
      case TankDrive: {
        double left = -driveHID.getLeftY();
        double right = -driveHID.getRightY();

        if (!straightMode) {
          robotDrive.tankDrive(left, right);
        } else {
          if (Math.abs(left) >= Math.abs(right)) {
            robotDrive.tankDrive(left, left);
          } else {
            robotDrive.tankDrive(right, right);
          }
        }
        break;
      }
      case ArcadeDrive: {
        double speed = -driveHID.getLeftY();
        double rotation = -driveHID.getLeftX();

        if (straightMode) {
          rotation = 0.0;
        }

        robotDrive.arcadeDrive(speed, rotation);
        break;
      }
      case CurvatureDrive: {
        double speed = -driveHID.getLeftY();
        double curvature = -driveHID.getRightX();

        if (straightMode) {
          curvature = 0.0;
        }

        robotDrive.curvatureDrive(speed, curvature, true);
        break;
      }
    }
  }

  void getTargetAngle() {
    if (driveHID.getLeftBumperButtonPressed()) {
      logf("Drive Straight start yaw:%.2f\n", yaw);
      targetAngle = Robot.yaw;
    }
    if (driveHID.getLeftBumperButtonReleased()) {
      logf("Drive Straight finish goal:%.2f yaw:%.2f\n", targetAngle, yaw);
      targetAngle = null;
    }
  }

  void driveStraight() {
    double error = Robot.yaw - targetAngle;
    if (error > 10) {
      logf("!!!!! Drive Straight error too positive diff:%.1f yaw:%.1f target:%.3f\n", error,
          yaw, targetAngle);
      error = 5;
    }
    if (error < -10) {
      logf("!!!!! Drive Straight error too negative diff:%.1f yaw:%.1f target:%.3f\n", error,
          yaw, targetAngle);
      error = -5;
    }
    // Adjsut speed if too fast
    double averageJoy = (rightJoy + leftJoy) / 1.0;
    double factor = error * Math.abs(averageJoy) * 0.035; // Was 0.045
    // Log drive straight data every 2.5 seconds
    if (Robot.count % 12 == 0) {
      logf("Drive Straight targ:%.2f yaw:%.2f err:%.2f avg:%.2f factor:%.2f Joy:<%.2f,%.2f>\n",
          targetAngle, yaw, error,
          averageJoy, factor, rightJoy, leftJoy);
    }
    leftJoy = averageJoy - factor;
    rightJoy = averageJoy + factor;
  }

}
