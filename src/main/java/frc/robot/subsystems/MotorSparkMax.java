package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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

public class MotorSparkMax extends SubsystemBase {
    private SparkMax motor;
    private SparkMax followMotor;
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

    public MotorSparkMax(MotorConfigs motorC) {
        this.motorC = motorC;
        myMotorSpark(motorC.name, motorC.id, motorC.followID, motorC.logging);
    }

    // public MotorSpark(String name, int id, int followId, boolean brakeMode, boolean invert) {
    //     motorC = new MotorConfigs(MotorTypes.SparkFlexMaxMotion);
    //     motorC.name = name;
    //     motorC.id = id;
    //     motorC.followID = followId;
    //     motorC.inverted = invert;
    //     myMotorSpark(name, id, followId, false);
    // }

    public MotorSparkMax(String name, int id, int followId, boolean brakeMode, boolean invert, MotorConfigs motorC) {
        this.motorC = motorC;
        motorC.name = name;
        motorC.id = id;
        motorC.followID = followId;
        motorC.inverted = invert;
        myMotorSpark(name, id, followId, false);
    }

    void myMotorSpark(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.followId = followId;
        myLogging = logging;
        logf("Start Spark Max %s id:%d\n", name, id);
        motor = new SparkMax(id, MotorType.kBrushless);
        setConfig(motor, -1, motorC);
        if (followId > 0) {
            followMotor = new SparkMax(followId, MotorType.kBrushless);
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

    @SuppressWarnings("removal")
    void setConfig(SparkMax motor, int followID, MotorConfigs motorC) {
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
        //config.closedLoop.maxOutput(motorC.maxOutput);
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
        //return motor.getClosedLoopController().
        return 0;
    }

    public void setSpeed(double speed) {
        if (lastSpeed == speed)
            return;
        motor.set(speed);
        lastSpeed = speed;
    }

    public void setVelocity(double velocity) {
        motor.getClosedLoopController().setReference(velocity, motorC.controlType);
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
        // logf("-------- Set motor %s at %.1f ticks\n", name, lastDesiredPosition);
       // motor.getClosedLoopController().setReference(position, .kMAXMotionPositionControl);
        motor.getClosedLoopController().setReference(position, motorC.controlType );
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
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logf(getMotorVCS(motor) + getMotorVCS(followMotor));
                } else {
                    logf(getMotorVCS(motor));
                }
            }
        }
    }

    public void periodic() {
        if (Robot.count % 10 == 0) {
            SmartDashboard.putNumber(name + " Pos", getPos());
            SmartDashboard.putNumber(name + " Revs", getPos() / motorC.positionConversionFactor);
            SmartDashboard.putNumber(name + " Cur", round2(motor.getOutputCurrent()));
            SmartDashboard.putNumber(name + " Vel", round2(getSpeed()));
            SmartDashboard.putNumber(name + " RPM", round2(getSpeed() / motorC.velocityConversionFactor / 60));
            logPeriodic();
        }
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

    private String getMotorVCS(SparkMax motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputCurrent = motor.getOutputCurrent();
            return String.format("%s motor volts:%.2f cur:%.2f power:%.2f sp:%.3f", name, bussVoltage,
                    outputCurrent, bussVoltage * outputCurrent, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getLastSpeed() {
        return motor.getAbsoluteEncoder().getVelocity();
    }

    public void logMotorVCS() {
        // if (Math.abs(motor.get()) < .01)
        // logf("%s motor not Running\n", name);
        if (Math.abs(getLastSpeed()) > .01) {
            if (followId > 0) {
                logf("%s motor volts<%.2f,%.2f> cur<%.2f,%.2f> sp<%.2f,%.2f>\n", name, motor.getBusVoltage(),
                        followMotor.getBusVoltage(), motor.getOutputCurrent(), followMotor.getOutputCurrent(),
                        motor.get(), followMotor.get());
            } else {
                logf("%s motor volts:%.2f cur:%.2f sp:%.2f\n", name, motor.getBusVoltage(), motor.getOutputCurrent(),
                        motor.get());
            }
        }
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