package frc.robot.subsystems;

import static frc.robot.utilities.Util.clip;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Jaguar;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Robot;
import frc.robot.Config.DriveType;

// The Jaguar works by recieving a PWM signal from a robot-controller.
// Depending on the range of the PWM signal - with 0 being full reverse, 127 being neutral, and 254 being full forward.

public class DrivetrainJaguar extends SubsystemBase {
    Jaguar rightMotor;
    Jaguar rightFollowMotor;
    Jaguar leftMotor;
    Jaguar leftFollowMotor;
    private double rightSpeed = 0;
    private double leftSpeed = 0;
    private Double targetAngle = null;
    private double rightJoy;
    private double leftJoy;
    private double yaw;
    private boolean showDriveLog = false;

    // Parameters for drive train
    private double rampTime = 1; // Change this value in setAggresiveMode() or setMildMode()
    private double slope = 0.0; // Slope of the ramp
    private double intercept = .0; // Offset of the ramp
    private double deadZone = 0.04;
    private boolean turboMode = false;
    private MySolenoidPCM shifter;
    private double sensitivity = .8;

    private XboxController driveHID;

    private double rampUp = .05;
    //private double rampUpTurbo = .05;
    private double rampDown = .05;
    //private double rampDownTurbo = .05;

    public DrivetrainJaguar(XboxController driveHID) {
        this.driveHID = driveHID;
        rightMotor = new Jaguar(Robot.config.driveRight);
        rightFollowMotor = new Jaguar(Robot.config.driveRightFollow);
        leftMotor = new Jaguar(Robot.config.driveLeft);
        leftFollowMotor = new Jaguar(Robot.config.driveLeftFollow);
        if (Robot.config.invertDrivetrain) {
            logf("Set Right and Left motors to  inverted\n");
            rightMotor.setInverted(false);
            leftMotor.setInverted(true);
            rightFollowMotor.setInverted(false);
            leftFollowMotor.setInverted(true);
        } else {
            logf("Set Right and Left motors to not inverted\n");
            rightMotor.setInverted(true);
            leftMotor.setInverted(false);
            rightFollowMotor.setInverted(true);
            leftFollowMotor.setInverted(false);
        }
        shifter = new MySolenoidPCM("Shifter", 1, 6, 7, true);
        shifter.pulseA();
    }

    public void setAggresiveMode() {
        logf("***** Set Aggreesive Mode\n");
        rampTime = .5;
        slope = 0;
        intercept = 0;
        deadZone = .04;
        setRampRate(rampTime);
    }

    public void setMildMode() {
        // Keanu liked a=.8, b=0.05, ramp=1.5
        logf("***** Set Mild Mode\n");
        rampTime = 1.5;
        slope = .8;
        intercept = 0.05;
        deadZone = .04;
        setRampRate(rampTime);
    }

    public void setRampRate(double rampTime) {
    }

    public void setSpeed(double right, double left) {
        setRightMotor(right * 1.2);
        setLeftMotor(left * 1.2);
    }

    public void setLeftMotor(double speed) {
        if (Math.abs(speed) < .005) {
            speed = 0;
        }
        leftMotor.set(speed);
        leftSpeed = speed;
    }

    public void forcePercentMode() {
    }

    public void setRightMotor(double speed) {
        if (Math.abs(speed) < .005)
            speed = 0;
        rightMotor.set(speed);
        rightSpeed = speed;
    }

    public double getLeftMotorSpeed() {
        return leftSpeed;
    }

    public double getRightMotorSpeed() {
        return rightSpeed;
    }

    @Override
    public void periodic() {
        if (driveHID.getRawButtonPressed(8)) {
            logf("Toggle Shifter\n");
            shifter.toggle();
        }
        yaw = Robot.yaw;
        if (Robot.count % 50 == 25) {
            SmartDashboard.putNumber("Right Sp", round2(rightMotor.get()));
            SmartDashboard.putNumber("Left Sp", round2(leftMotor.get()));
        }
        if (Robot.count % 50 == 0) {
            SmartDashboard.putNumber("Right F Sp", round2(rightFollowMotor.get()));
            SmartDashboard.putNumber("Left F Sp", round2(leftFollowMotor.get()));
        }
        if (driveHID.getLeftBumperButtonPressed()) {
            logf("Drive Straight start yaw:%.2f\n", yaw);
            targetAngle = Robot.yaw;
        }
        if (driveHID.getLeftBumperButtonReleased()) {
            logf("Drive Straight finish goal:%.2f yaw:%.2f\n", targetAngle, yaw);
            targetAngle = null;
        }
        if (Config.driveType == DriveType.MildArcade || Config.driveType == DriveType.AggressiveArcade) {
            arcadeMode();
            return;
        }
        double sensitivity = 1.0;

        rightJoy = driveHID.getRightY();
        leftJoy = driveHID.getLeftY();

        if (targetAngle != null) {
            // If Drive straight active make adjustments
            driveStraight();
        }

        rightJoy *= sensitivity;
        leftJoy *= sensitivity;

        // Adjust the ramp rate to make it easier to drive
        // Ramping will also put less strain on the motors
        rightJoy = ramp(rightJoy, rightSpeed, "Right");
        leftJoy = ramp(leftJoy, leftSpeed, "Left");

        showDriveLog = Math.abs(rightJoy) > .06 || Math.abs(leftJoy) > .06;
        if (showDriveLog && Robot.count % 100 == 0) {
            logf("Joy L:%.2f R:%.2f\n", leftJoy, rightJoy);
        }
        setLeftMotor(leftJoy);
        setRightMotor(rightJoy);
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

    double rampCubed(double joy, double speed, String side) {
        double result;
        result = 0; // Set to 0 for default case i.e. in dead zone
        if (joy > deadZone) {
            result = intercept + (1 - intercept) * (slope * joy * joy * joy + (1 - slope) * joy);
        }
        if (joy < -deadZone) {
            result = -intercept + (1 - intercept) * (slope * joy * joy * joy + (1 - slope) * joy);
        }
        double act = 0;
        if (side == "Right")
            act = rightMotor.get();
        if (side == "Left")
            act = leftMotor.get();
        if (Math.abs(act) > 0.01) {
            if (Math.abs(lastSpeed - speed) > .05 && side == "Right") {
                logf("Ramp Cubed side:%s joy:%.2f result:%.2f last requested:%.2f motor speed:%.2f slope%.2f\n",
                        side, joy, result, speed, act, slope);
                lastSpeed = speed;
            }
        }
        return result;
    }

    boolean rampCub = true;
    double lastSpeed = 0;

    public double ramp(double joy, double speed, String side) {
        if (rampCub)
            return rampCubed(joy, speed, side);
        double error = Math.abs(speed - joy);
        double newJoy = joy;
        if (error < .04)
            // If small difference simply return joystick value
            return joy;
        if (turboMode) {
            if (joy > speed) {
                newJoy = speed + rampUp;
            } else {
                newJoy = speed - rampDown;
            }
        } else {
            if (joy > speed) {
                newJoy = speed + rampUp;
            } else {
                newJoy = speed - rampDown;
            }
        }
        // Log ramp data if it changed -- only do for right motor to avoid lots of logs
        if (Math.abs(lastSpeed - speed) > .05 && side == "Right") {
            logf("Ramp side:%s in joy:%.2f new joy:%.2f speed:%.2f\n", side, joy, newJoy, speed);
            lastSpeed = speed;
        }
        return newJoy;
    }

    public double rampExp(double joy, double speed, String side) {
        double error = Math.abs(speed - joy);
        if (error < .04)
            return joy;
        boolean joyIsNegative = joy < 0;
        double newJoy = 0.2 * (Math.exp(1.75 * (Math.abs(joy) - 1)));
        newJoy = (joyIsNegative ? -newJoy : newJoy);
        logf("Exp Ramp side:%s in joy:%.2f new joy:%.2f speed:%.2f\n", side, joy, newJoy, speed);
        return newJoy;
    }

    private double correctForDeadZone(double speed) {
        if (Math.abs(speed) < deadZone) {
            return 0;
        }
        return speed;
    }

    private void arcadeMode() {
        double yValue = driveHID.getLeftY() * -1;
        double xValue = driveHID.getLeftX() * -1;
        yValue = correctForDeadZone(yValue) * sensitivity;
        xValue = correctForDeadZone(xValue) * sensitivity;

        double leftPower = yValue - xValue;
        double rightPower = yValue + xValue;

        leftMotor.set(clip(leftPower, -1.0, 1.0));
        rightMotor.set(clip(rightPower, -1.0, 1.0));

        leftFollowMotor.set(clip(leftPower, -1.0, 1.0));
        rightFollowMotor.set(clip(rightPower, -1.0, 1.0));

        if (Math.abs(yValue) > .1 && Math.abs(yValue) > .1) {
            if (Robot.count % 5 == 0) {
                logf("Arcade agressive Drive Speed  r:%.3f l:%.3f\n", rightPower, leftPower);
            }
        }
    }
}
