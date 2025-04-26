package frc.robot.subsystems;

import static frc.robot.Robot.robotContainer;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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

public class MotorSparkMax extends SubsystemBase {
    private SparkMax motor;
    private SparkMax followMotor;
    private String name;
    private int followId;
    private boolean brakeMode;
    private double lastSpeed;
    private SparkLimitSwitch forwardSwitch;
    private SparkLimitSwitch reverseSwitch;
    private boolean myLogging = true;
    private double lastDesiredPosition;
    private RelativeEncoder relEncoder;
    private double lastPos = 0;
    private SparkMaxConfig motorConfig;
    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;
    private boolean testMode = false;

    public MotorSparkMax(String name, int id, int followId, boolean brakeMode, boolean invert) {
        this.name = name;
        this.followId = followId;
        this.brakeMode = brakeMode;
        myMotorSpark(name, id, followId, false);
    }

    void myMotorSpark(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.followId = followId;
        myLogging = logging;
        logf("Start Spark Max %s id:%d\n", name, id);
        motor = new SparkMax(id, MotorType.kBrushless);
        setConfig(motor);
        if (followId > 0) {
            followMotor = new SparkMax(followId, MotorType.kBrushless);
            setConfig(followMotor);
        }
        setBrakeMode(brakeMode);
        relEncoder = motor.getEncoder();
        relEncoder.setPosition(0.0);

        if (followId > 0)
            logf("Created %s dual motors ids:<%d,%d> firmware:<%s,%s> \n", name, id,
                    followId, motor.getFirmwareString(), followMotor.getFirmwareString());
        else
            logf("Created %s motor id:%d firmware:%s \n", name, id,
                    motor.getFirmwareString());
    }

    public void setTestMode(boolean value) {
        testMode = value;
    }

    public void setLogging(boolean value) {
        myLogging = value;
    }

    private void setConfig(SparkMax motor) {
        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */
        motorConfig = new SparkMaxConfig();

        motorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        motorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        motorConfig.encoder
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);

        // Configure the closed loop controller. We want to make sure we set the
        // feedback sensor as the primary encoder.
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed
                // loop slot, as it will default to slot 0.
                .p(0.2)  // was .2
                .i(0)
                .d(1)  // was 1
                .outputRange(-1, 1);

        motorConfig.closedLoop
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motorConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for position control. We don't need to pass
                // a closed loop slot, as it will default to slot 0.
                .maxVelocity(4000)
                .maxAcceleration(3000)
                .allowedClosedLoopError(.05)
                // Set MAXMotion parameters for velocity control in slot 1
                .maxAcceleration(3000, ClosedLoopSlot.kSlot1) // Was 500
                .maxVelocity(6000, ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot1);

        /*
         * Apply the configuration to the SPARK MAX.
         *
         * kPersistParameters is used to ensure the configuration is not lost when
         * the SPARK MAX loses power. This is useful for power cycles that may occur
         * mid-operation.
         */
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setBrakeMode(boolean value) {
        motorConfig.idleMode(value ? IdleMode.kBrake : IdleMode.kCoast);
    }

    // @SuppressWarnings("removal")
    // void setConfig(SparkMax motor, int followID, MotorConfigs motorC) {
    // SparkMaxConfig config = new SparkMaxConfig();
    // if (followID > 0) {
    // config.follow(followId);
    // }
    // config.inverted(motorC.inverted);
    // config.idleMode(motorC.brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    // config.encoder.positionConversionFactor(motorC.positionConversionFactor);
    // config.encoder.velocityConversionFactor(motorC.velocityConversionFactor);
    // config.limitSwitch.forwardLimitSwitchEnabled(motorC.forwardLimit);
    // config.limitSwitch.reverseLimitSwitchEnabled(motorC.reverseLimit);
    // config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // config.closedLoop.pidf(motorC.kP, motorC.kI, motorC.kD, motorC.kFF);
    // //config.closedLoop.maxOutput(motorC.maxOutput);
    // config.closedLoop.smartMotion.maxVelocity(motorC.maxVelocity);
    // config.closedLoop.smartMotion.maxAcceleration(motorC.maxAcceration);
    // config.closedLoop.maxMotion.maxVelocity(motorC.maxVelocity);
    // config.closedLoop.maxMotion.maxAcceleration(motorC.maxAcceration);
    // config.smartCurrentLimit(motorC.currentLimit);
    // config.openLoopRampRate(motorC.openLoopRampRate);
    // config.closedLoopRampRate(motorC.openLoopRampRate);
    // motor.configure(config, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // }

    public double getSpeed() {
        return relEncoder.getVelocity();
    }

    public double getError() {
        return Math.abs(lastDesiredPosition - getPos());
    }

    public void setSpeed(double speed) {
        if (lastSpeed == speed)
            return;
        motor.set(speed);
        lastSpeed = speed;
    }

    public void setVelocity(double velocity) {
        motor.getClosedLoopController().setReference(velocity, SparkFlex.ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot1);
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
            SmartDashboard.putNumber(name + " Pos", getPos());  // display position in rotations
            SmartDashboard.putNumber(name + " Revs", getPos() / positionConversionFactor); //display position based on conversion factor
            SmartDashboard.putNumber(name + " Cur", round2(motor.getOutputCurrent())); //display motor current
            SmartDashboard.putNumber(name + " Vel", round2(getSpeed())); //display speed in  revolutions per minute
            SmartDashboard.putString("Mode", mode.toString()); //display mode 
            SmartDashboard.putNumber(name + " RPM", round2(getSpeed() / velocityConversionFactor / 60)); //display speed in  revolutions per minute
            logPeriodic();
            if (testMode)
                testCases();
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

    private String getMotorVCS(SparkMax motor) {
        double bussVoltage = motor.getBusVoltage();
        double outputCurrent = motor.getOutputCurrent();
        return String.format("%s motor volts:%.2f cur:%.2f velocity:%.2f power:%.2f sp:%.3f", name, bussVoltage,
                outputCurrent, motor.getAbsoluteEncoder().getVelocity(), bussVoltage * outputCurrent, lastSpeed);
    }

    public double getLastSpeed() {
        return motor.getAbsoluteEncoder().getVelocity();
    }

    public void logMotorVCS() {
        if (Math.abs(getLastSpeed()) > .01) {
            logf("%s\n", getMotorVCS(motor));
            if (followId > 0) 
                logf("%s\n", getMotorVCS(followMotor));
        }
    }

    // public void logAllMotor() {
    //     if (followId > 0) {
    //         logf("%s,bv,%.2f,%.2f,av,%.2f,%.2f,oc,%.2f,%.2f,sp,%.2f,%.2f,enc,%.0f,%.0f,vel,%.3f,%.3f\n", name,
    //                 motor.getBusVoltage(), followMotor.getBusVoltage(), motor.getAppliedOutput(),
    //                 followMotor.getAppliedOutput(), motor.getOutputCurrent(), followMotor.getOutputCurrent(),
    //                 motor.get(), followMotor.get(), relEncoder.getPosition(), relEncoderFollow.getPosition(),
    //                 relEncoder.getVelocity(), relEncoderFollow.getVelocity());
    //     } else {
    //         logf("%s motor volts:%.2f cur:%.2f sp:%.2f\n", name, motor.getBusVoltage(), motor.getOutputCurrent(),
    //                 motor.get());
    //     }
    // }

    enum Modes {
        POSITION, VELOCITY, MOTIONMAGIC, SPEED;

        public Modes next() {
            Modes[] values = Modes.values();
            int nextOrdinal = (this.ordinal() + 1) % values.length;
            return values[nextOrdinal];
        }
    }

    Modes mode = Modes.POSITION;
    boolean lastStart = false;

    void testCases() {
        CommandXboxController driveController = RobotContainer.driveController;
        double value;
        // Hiting the start button moves to the next control method
        boolean start = driveController.start().getAsBoolean();
        if (start && !lastStart) {
            setVelocity(0.0);
            setSpeed(0.0);
            setPos(0.0);
            mode = mode.next(); // Get the next mode
            logf("New Test Mode:%s\n", mode);
        }
        lastStart = start;
        switch (mode) {
            case POSITION:
                value = driveController.getHID().getPOV() / 10.0;
                if (value >= 0.0) {
                    setPos(value);
                    logf("Flex set position:%.6f\n", value);
                }
                break;
            case VELOCITY:
                value = driveController.getHID().getPOV() * 20;
                if (value >= 0) {
                    setVelocity(value);
                    logf("Flex set velocity:%.2f\n", value);
                }
                break;
            case MOTIONMAGIC:
                break;
            case SPEED:
                double mySpeed = robotContainer.getSpeedFromTriggers();
                if (mySpeed > 0.05)
                    logf("Set Test speed:%.2f\n", mySpeed);
                setSpeed(mySpeed);
                break;
        }
    }
}