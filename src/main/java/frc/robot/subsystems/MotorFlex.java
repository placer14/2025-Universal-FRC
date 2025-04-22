package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * SPARK MAX controllers are initialized over CAN by constructing a SparkMax
 * object
 *
 * Look to:
 * 
 * https://docs.revrobotics.com/brushless/revlib/revlib-overview/migrating-to-revlib-2025
 * https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/package-summary
 * https://www.chiefdelphi.com/t/rev-robotics-2024-2025/471083/139
 * 
 * for 2025 information
 *
 */

public class MotorFlex extends SubsystemBase implements MotorDef {
    private SparkFlex motor;
    private SparkFlex followMotor;
    private String name;
    private int followId;
    private double lastSpeed;
    private SparkLimitSwitch forwardSwitch;
    private SparkLimitSwitch reverseSwitch;
    private boolean myLogging = true;
    private double lastDesiredPosition;
    public RelativeEncoder relEncoder;
    public RelativeEncoder relEncoderFollow;
    private double lastPos = 0;
    private MotorConfigs motorC;

    public MotorFlex(MotorConfigs motorC) {
        this.motorC = motorC;
        myMotorFlex(motorC.name, motorC.id, motorC.followID, motorC.logging);
    }

    public MotorFlex(String name, int id, int followId, boolean brakeMode, boolean invert, MotorConfigs motorC) {
        this.motorC = motorC;
        motorC.name = name;
        motorC.id = id;
        motorC.followID = followId;
        motorC.inverted = invert;
        myMotorFlex(name, id, followId, false);
    }

    void myMotorFlex(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.followId = followId;
        myLogging = logging;
        logf("Start Spark Max %s id:%d\n", name, id);
        motor = new SparkFlex(id, MotorType.kBrushless);
        setConfig(motor, -1, motorC);
        if (followId > 0) {
            followMotor = new SparkFlex(followId, MotorType.kBrushless);
            setConfig(followMotor, followId, motorC);
            relEncoderFollow = followMotor.getEncoder();
        }
        relEncoder = motor.getEncoder();
        relEncoder.setPosition(0.0);

        if (followId > 0)
            logf("Created %s dual motors ids:<%d,%d> firmware:<%s,%s> \n", name, id,
                    followId, motor.getFirmwareString(), followMotor.getFirmwareString());
        else
            logf("Created %s motor id:%d firmware:%s \n", name, id,
                    motor.getFirmwareString());
    }

    public void setLogging(boolean value) {
        logf("Set Flex Logging:%b\n", value);
        myLogging = value;
    }

    public void enableLimitSwitch(boolean forward, boolean reverse) {
    };

    public void setInverted(boolean invert) {
    };

    public double getActualVelocity() {
        return 0.0;
    };

    public void forcePercentMode() {
    };

    /*
     * Peak Current and Duration must be exceeded before current limit is activated.
     * When activated, current will be limited to Continuous Current. Set Peak
     * Current params to 0 if desired behavior is to immediately current-limit.
     */
    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
    };

    public void updateSmart() {
    };

    public void setSpeedAbsolute(double speed) {
    };

    public void setVelocityPID(PID pid) {
    };

    public void PIDToMotor(PID pid, int slot, int timeout) {
    };

    public String getMotorVCS() {
        return "";
    };

    public void setSensorPhase(boolean phase) {
    };

    // Config the sensor used for Primary PID and sensor direction
    public void setPositionPID(int pidIdx, PID pid) {

    };

    public void setRampClosedLoop(double rate) {
    };

    public void setRampOpenLoop(double rate) {
    };

    @SuppressWarnings("removal")
    void setConfig(SparkFlex motor, int followID, MotorConfigs motorC) {
        SparkMaxConfig config = new SparkMaxConfig();
        if (followID > 0) {
            config.follow(followId);
        }
        config.inverted(motorC.inverted);
        config.idleMode(motorC.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        config.encoder.positionConversionFactor(motorC.positionConversionFactor);
        config.encoder.velocityConversionFactor(motorC.velocityConversionFactor);
        config.limitSwitch.forwardLimitSwitchEnabled(motorC.forwardLimit);
        config.limitSwitch.reverseLimitSwitchEnabled(motorC.reverseLimit);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pidf(motorC.kP, motorC.kI, motorC.kD, motorC.kFF);
        // config.closedLoop.maxOutput(motorC.maxOutput);
        config.closedLoop.smartMotion.maxVelocity(motorC.maxVelocity);
        config.closedLoop.smartMotion.maxAcceleration(motorC.maxAcceration);
        config.closedLoop.maxMotion.maxVelocity(motorC.maxVelocity);
        config.closedLoop.maxMotion.maxAcceleration(motorC.maxAcceration);
        config.smartCurrentLimit(motorC.currentLimit);
        config.openLoopRampRate(motorC.openLoopRampRate);
        config.closedLoopRampRate(motorC.openLoopRampRate);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getSpeed() {
        return relEncoder.getVelocity();
    }

    public double getError() {
        return Math.abs(lastDesiredPosition - getPos());
    }

    public double getErrorPID() {
        // return motor.getClosedLoopController().
        return 0;
    }

    public void setSpeed(double speed) {
        if (lastSpeed == speed)
            return;
        motor.set(speed);
        lastSpeed = speed;
    }

    public void setVelocity(double velocity) {
        motor.getClosedLoopController().setReference(velocity, SparkFlex.ControlType.kVelocity);
    }

    public void stopMotor() {
        motor.stopMotor();
        lastSpeed = 0;
    }

    public double getMotorVoltage() {
        return motor.getAppliedOutput();
    }

    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    public double getFollowCurrent() {
        if (followMotor == null) {
            return 0;
        }
        return followMotor.getOutputCurrent();
    }

    public double getPos() {
        return relEncoder.getPosition();
    }

    public void setPos(double position) {
        lastDesiredPosition = position;
        motor.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    public void zeroEncoder() {
        relEncoder.setPosition(0);
    }

    public void setEncoderPosition(double position) {
        relEncoder.setPosition(position);
    }

    public boolean getForwardLimitSwitch() {
        return forwardSwitch.isPressed();
    }

    public boolean getReverseLimitSwitch() {
        return reverseSwitch.isPressed();
    }

    public void logPeriodic() {
        double pos = getPos();
        if (Math.abs(pos - lastPos) > .05) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logMotorVCS();
                    logMotorVCS(followMotor);
                } else {
                    logMotorVCS();
                }
            }
        }
    }

    @Override
    public void periodic() {
        if (Robot.count % 10 == 0) {
            SmartDashboard.putNumber(name + " Pos", getPos());
            SmartDashboard.putNumber(name + " Revs", getPos() / motorC.positionConversionFactor);
            SmartDashboard.putNumber(name + " Cur", round2(motor.getOutputCurrent()));
            SmartDashboard.putNumber(name + " Vel", round2(getSpeed()));
            SmartDashboard.putNumber(name + " RPM", round2(getSpeed() / motorC.velocityConversionFactor / 60));
            logPeriodic();
            testCases();
        }
    }

    enum Modes {
        SPEED, POSTION, VELOCITY, MOTIONMAGIC
    }

    Modes mode = Modes.SPEED;

    void testCases() {
        double kP, kI, kD, kIz, kFF;
        SparkMaxConfig config;
        double value;
        CommandXboxController driveController = RobotContainer.driveController;
        if (driveController.start().getAsBoolean()) {
            motor.stopMotor();
            switch (mode) {
                case SPEED:
                    // Setup for testing the position PID
                    mode = Modes.POSTION;
                    config = new SparkMaxConfig();
                    kP = 0.1;
                    kI = 1e-4;
                    kD = 1;
                    kIz = 0;
                    kFF = 0;
                    config.closedLoop.pidf(kP, kI, kD, kFF);
                    config.closedLoop.iZone(kIz);
                    break;
                case POSTION:
                    // Setup for testing the velocity PID
                    mode = Modes.VELOCITY;
                    config = new SparkMaxConfig();
                    kP = 6e-5;
                    kI = 0;
                    kD = 0;
                    kIz = 0;
                    kFF = 0.000015;
                    config.closedLoop.pidf(kP, kI, kD, kFF);
                    config.closedLoop.iZone(kIz);
                    break;
                case VELOCITY:
                    mode = Modes.MOTIONMAGIC;
                    break;
                case MOTIONMAGIC:
                    mode = Modes.SPEED;
                    break;
            }
            logf("New Test Mode:%s\n", mode);
        }
        switch (mode) {
            case SPEED:
                value = driveController.getLeftTriggerAxis();
                if (value > .05) {
                    if (driveController.getHID().getLeftBumper())
                        value = -value;
                    setSpeed(value);
                } else {
                    stopMotor();
                }
                break;
            case POSTION:
                value = driveController.getHID().getPOV() / 45.0;
                if (value >= 0.0) {
                    setPos(value);
                    logf("Flex set position:%.2f\n", value);
                }
                break;
            case VELOCITY:
                value = driveController.getHID().getPOV();
                if (value >= 0.02) {
                    setVelocity(value);
                    logf("Flex set velocity:%.2f\n", value);
                }
                break;
            case MOTIONMAGIC:
                break;

        }

        // double value = driveController.getRightTriggerAxis();
        // if (Math.abs(value) > .02) {
        // setSpeed(value);
        // }
        // if (driveController.povUp().getAsBoolean()) {
        // setVelocity(10);
        // }
        // if (driveController.povDown().getAsBoolean()) {
        // setVelocity(0);
        // }
        // if (driveController.povRight().getAsBoolean()) {
        // setPos(2);
        // }
        // if (driveController.povLeft().getAsBoolean()) {
        // setPos(0);
        // }

    }

    void testTimes() {
        long startTime = RobotController.getFPGATime();
        double sp = motor.get();
        long endTime1 = RobotController.getFPGATime();
        double cur = motor.getOutputCurrent();
        long endTime2 = RobotController.getFPGATime();
        double enc = relEncoder.getPosition();
        long endTime3 = RobotController.getFPGATime();
        logf("%s motor sp:%.2f pos:%.0f\n", name, sp, cur, enc);
        long endTime = RobotController.getFPGATime();
        System.out.printf("get:%d cur:%d enc:%d log:%d total:%d\n", endTime1 - startTime, endTime2 - endTime1,
                endTime3 - endTime2, endTime - endTime3, endTime - startTime);
    }

    public void logPeriodicSlow() {
        if (followId > 0) {
            logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, motor.get(), motor.getOutputCurrent(),
                    motor.getMotorTemperature(), relEncoder.getVelocity(), relEncoder.getPosition());
            logf("%s rear  sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, followMotor.get(),
                    followMotor.getOutputCurrent(), followMotor.getMotorTemperature(), relEncoderFollow.getVelocity(),
                    relEncoderFollow.getPosition());
        } else {
            logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, motor.get(), motor.getOutputCurrent(),
                    motor.getMotorTemperature(), relEncoder.getVelocity(), relEncoder.getPosition());
        }
    }

    public void logMotorVCS() {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputCurrent = motor.getOutputCurrent();
            logf("%s motor volts:%.2f cur:%.2f power:%.2f sp:%.3f\n", name, bussVoltage,
                    outputCurrent, bussVoltage * outputCurrent, lastSpeed);
        }
    }

    public void logMotorVCS(SparkFlex motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputCurrent = motor.getOutputCurrent();
            logf("%s motor volts:%.2f cur:%.2f power:%.2f sp:%.3f\n", name, bussVoltage,
                    outputCurrent, bussVoltage * outputCurrent, lastSpeed);
        }
    }

    public double getLastSpeed() {
        return motor.getAbsoluteEncoder().getVelocity();
    }

    public void logAllMotor() {
        if (followId > 0) {
            logf("%s,bv,%.2f,%.2f,av,%.2f,%.2f,oc,%.2f,%.2f,sp,%.2f,%.2f,enc,%.0f,%.0f,vel,%.3f,%.3f\n", name,
                    motor.getBusVoltage(), followMotor.getBusVoltage(), motor.getAppliedOutput(),
                    followMotor.getAppliedOutput(), motor.getOutputCurrent(), followMotor.getOutputCurrent(),
                    motor.get(), followMotor.get(), relEncoder.getPosition(), relEncoderFollow.getPosition(),
                    relEncoder.getVelocity(), relEncoderFollow.getVelocity());
        } else {
            logf("%s motor volts:%.2f cur:%.2f sp:%.2f\n", name, motor.getBusVoltage(), motor.getOutputCurrent(),
                    motor.get());
        }
    }
}