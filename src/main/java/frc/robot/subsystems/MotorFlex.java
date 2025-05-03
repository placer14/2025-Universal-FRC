package frc.robot.subsystems;

import static frc.robot.Robot.robotContainer;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
    private RelativeEncoder relEncoder;
    private double lastPos = 0;
    private double positionConversionFactor = 1;
    private double velocityConversionFactor = 1;
    private boolean enableTestMode = false;
    private SparkMaxConfig motorConfig;
    //private ClosedLoopConfig closedLoopConfig;
    LedSubsystem leds;

    public MotorFlex(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.followId = followId;
        this.myLogging = logging;
        motor = new SparkFlex(id, MotorType.kBrushless);
        setConfig(motor);

        // setConfig(motor, -1, motorC);
        if (followId > 0) {
            followMotor = new SparkFlex(followId, MotorType.kBrushless);
            setConfig(motor);
        }
        setBrakeMode(true);
        relEncoder = motor.getEncoder();
        relEncoder.setPosition(0.0);

        if (followId > 0)
            logf("Created %s dual motors ids:<%d,%d> firmware:<%s,%s> \n", name, id,
                    followId, motor.getFirmwareString(), followMotor.getFirmwareString());
        else
            logf("Created %s motor id:%d firmware:%s \n", name, id,
                    motor.getFirmwareString());
    }

    private void setConfig(SparkFlex motor) {
        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */
        motorConfig = new SparkMaxConfig();
        //closedLoopConfig = motorConfig.closedLoop;

        motorConfig.encoder.positionConversionFactor(positionConversionFactor);
        motorConfig.encoder.velocityConversionFactor(velocityConversionFactor);
        motorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        motorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        motorConfig.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);

        // Configure the closed loop controller. We want to make sure we set the
        // feedback sensor as the primary encoder.
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control in slot 0.
                .p(0.2, ClosedLoopSlot.kSlot0)
                .i(0, ClosedLoopSlot.kSlot0)
                .d(1, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        motorConfig.closedLoop
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1) // Note: 5767 is max velocity
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motorConfig.closedLoop
                // Set PID values for Magic motion position control in slot 2
                .p(.4, ClosedLoopSlot.kSlot2)
                .i(0, ClosedLoopSlot.kSlot2)
                .d(0, ClosedLoopSlot.kSlot2)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot2);

        motorConfig.closedLoop
                // Set PID values for Magic motion velocty in slot 3
                .p(0.0001, ClosedLoopSlot.kSlot3)
                .i(0, ClosedLoopSlot.kSlot3)
                .d(0, ClosedLoopSlot.kSlot3)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot3) // Note 5767 is max speed for Velocity PIDs
                .outputRange(-1, 1, ClosedLoopSlot.kSlot3);

        motorConfig.closedLoop.maxMotion
                // Set Motion parameter for Velocity Control
                .maxVelocity(5000, ClosedLoopSlot.kSlot1)
                .maxAcceleration(5000, ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot1)

                // Set MAXMotion parameters for position control in slot 2
                .maxAcceleration(4000, ClosedLoopSlot.kSlot2) // Was 500
                .maxVelocity(4000, ClosedLoopSlot.kSlot2)
                .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot2)

                // Set MAXMotion parameters for velocity control in slot 3
                .maxAcceleration(3000, ClosedLoopSlot.kSlot3) // Was 500
                .maxVelocity(6000, ClosedLoopSlot.kSlot3)
                .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot3);

        /*6
         * Apply the configuration to the SPARK MAX.
         *
         * kResetSafeParameters is used to get the SPARK MAX to a known state. This
         * is useful in case the SPARK MAX is replaced.
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

    public void setLogging(boolean value) {
        logf("Set Flex Logging:%b\n", value);
        myLogging = value;
    }

    public void setTestMode(boolean value) {
        logf("Set Flex Test Mode:%b\n", value);
        enableTestMode = value;
    }

    public void enableLimitSwitch(boolean forward, boolean reverse) {
    }

    public void setInverted(boolean invert) {
    }

    public double getActualVelocity() {
        return relEncoder.getVelocity();
    }

    public double getLastSpeed() {
        return lastSpeed;
    }

    public void forcePercentMode() {
    }

    /*
     * Peak Current and Duration must be exceeded before current limit is activated.
     * When activated, current will be limited to Continuous Current. Set Peak
     * Current params to 0 if desired behavior is to immediately current-limit.
     */
    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
    }

    public void updateSmart() {
    }

    public void setSpeedAbsolute(double speed) {
    }

    public void setVelocityPID(PID pid, int slot, FeedbackDevice feedbackDevice) {
    }

    public void PIDToMotor(PID pid, int slot, int timeout) {
    }

    public String getMotorVCS() {
        return getMotorVCS(motor);
    }

    public String getMotorVCS(SparkFlex motor) {
        return String.format("%s motor volts:%.2f cur:%.2f vel:%.2f sp:%.2f\n", name, motor.getBusVoltage(),
                motor.getOutputCurrent(),
                motor.getEncoder().getVelocity(), motor.get());
    }

    public void setSensorPhase(boolean phase) {
    }

    // Config the sensor used for Primary PID and sensor direction
    public void setPositionPID(int pidIdx, PID pid) {
    }

    public void setRampClosedLoop(double rate) {
    }

    public void setRampOpenLoop(double rate) {
    }

    public double getSpeed() {
        return motor.get();
    }

    public double getError() {
        return Math.abs(lastDesiredPosition - getPos());
    }

    public double getErrorPID() {
        return getError();
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

    public void setMagicPositon(double position) {
        //motor.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl,  ClosedLoopSlot.kSlot3);
        motor.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl,  ClosedLoopSlot.kSlot2);
    }

    public void setMagicVelocity(double velocity) {
        motor.getClosedLoopController().setReference(velocity, ControlType.kMAXMotionVelocityControl,  ClosedLoopSlot.kSlot3);
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
        if (Robot.count % 1 == 0) {
            SmartDashboard.putNumber("Pos", getPos());
            SmartDashboard.putNumber("Rot", getPos() / positionConversionFactor);
            SmartDashboard.putNumber("Cur", round2(motor.getOutputCurrent()));
            SmartDashboard.putNumber("Vel", round2(getActualVelocity()));
            SmartDashboard.putNumber("RPM", round2(getActualVelocity() / velocityConversionFactor / 60));
            SmartDashboard.putString("Mode", mode.toString());
            SmartDashboard.putNumber("Err", getError());
            logPeriodic();
            if (enableTestMode)
                testCases();
        }
    }

    public void setLeds(LedSubsystem leds) {
        this.leds = leds;
    }

    enum Modes {
        POSITION, VELOCITY, POSMAGIC, VELMAGIC, SPEED;

        public Modes next() {
            Modes[] values = Modes.values();
            int nextOrdinal = (this.ordinal() + 1) % values.length;
            return values[nextOrdinal];
        }
    }

    Modes mode = Modes.POSITION;
    boolean lastStart = false;
    double setP = 0;

    void testCases() {
        CommandXboxController driveController = RobotContainer.driveController;
        double value = 0;
        // Hiting the start button moves to the next control method
        boolean start = driveController.start().getAsBoolean();
        if (start && !lastStart) {
            setSpeed(0.0);
            relEncoder.setPosition(0.0);
            mode = mode.next(); // Get the next mode
            logf("New Test Mode:%s\n", mode);
        }
        lastStart = start;
        switch (mode) {
            case POSITION:
                value = driveController.getHID().getPOV() / 22.5;
                if (value >= 0.0) {
                    setPos(value);
                    setP = value;
                    logf("Flex set position:%.2f\n", value);
                }
                break;
            case VELOCITY:
                value = driveController.getHID().getPOV() * 20;
                if (value >= 0) {
                    setVelocity(value);
                    setP = value;
                    logf("Flex set velocity:%.2f\n", value);
                }
                break;
            case POSMAGIC:
                value = driveController.getHID().getPOV() / 5.0;
                if (value >= 0) {
                    setMagicPositon(value);
                    setP = value;
                    logf("Flex set positon Magic Motion:%.2f\n", value);
                }
                break;
            case VELMAGIC:
                value = driveController.getHID().getPOV() * 20;
                if (value >= 0) {
                    setMagicVelocity(value);
                    setP = value;
                    logf("Flex set velocity Maguc Motion:%.2f\n", value);
                }
                break;
            case SPEED:
               value = robotContainer.getSpeedFromTriggers();
                if (value > 0.05)
                    logf("Set Test speed:%.2f\n", value);
                    setP = value;
                setSpeed(value);
                break;
        }
        SmartDashboard.putNumber("SetP", setP);
        leds.setAllColors(0, 0, 0);
        leds.setRangeOfColor(0, mode.ordinal(), 0, 127, 0);
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

    public void logMotorVCS() {
        logMotorVCS(motor);
    }

    public void logMotorVCS(SparkFlex motor) {
        if (Math.abs(lastSpeed) > .02) {
            logf("%s", getMotorVCS(motor));
        }
    }

    public void logAllMotor() {
        logMotorVCS(motor);
        if (followId > 0)
            logMotorVCS(followMotor);
    }
}