package frc.robot.subsystems;

import static frc.robot.Robot.count;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// setSensorPhase() should change the direction reported in the getSelectedSensor*() methods 
// (but not the SensorCollection methods).
// It should also change the direction reported for "PID0" in a self-test snapshot, 
// but not the position reported by "Quad/MagEnc(rel)" 

public class MotorFX extends SubsystemBase implements MotorDef {
    private TalonFX motor;
    private TalonFX followMotor;
    private String name;
    private int id;
    private int followId;
    private double lastSpeed = 0;
    private double lastPos = 0;
    public ErrorCode errorCode;
    public ErrorCode errorCodeFollow;
    /* Start at position 0, use slot 0 */
    // private final PositionVoltage m_positionVoltage = new
    // PositionVoltage(0).withSlot(0);
    /* Start at position 0, use slot 1 */
    // private final PositionTorqueCurrentFOC m_positionTorque = new
    // PositionTorqueCurrentFOC(0).withSlot(1);

    private boolean sensorPhase = false;
    private boolean motorInvert = false;

    private final MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

    MotorFX(String name, int id, int followId) {
        this.name = name;
        this.id = id;
        this.followId = followId;
        motor = new TalonFX(this.id);
        TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();
        talonFXConfigurator.apply(new TalonFXConfiguration());
        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(motorConfigs);

        if (errorCode != ErrorCode.OK) {
            logf("????????? Motor %s Error: %s ??????????\n", name, errorCode);
        }
        if (followId > 0) {
            followMotor = new TalonFX(followId);
            /* Set follower to follow leader */
            followMotor.setControl(new Follower(motor.getDeviceID(), false));
            if (errorCode != ErrorCode.OK) {
                logf("????????? Follow Motor %s Error: %s ??????????\n", name, errorCode);
                followId = -followId;
            }

        }
        motor.setPosition(0);
        if (followId > 0)
            logf("Created %s motor ids:<%d,%d> firmware:<%d,%d> voltage:<%.1f,%.1f>\n", name, id, followId,
                    motor.getVersion(), followMotor.getVersion(), motor.getMotorVoltage(),
                    followMotor.getMotorVoltage());
        else
            logf("Created %s motor id:%d firmware:%d voltage:%.1f\n", name, id, motor.getVersion(),
                    motor.getMotorVoltage());

    }

    public String getName() {
        return name;
    }

    public double getPos() {
        return motor.getRotorPosition().getValueAsDouble();
    }


   
    public void enableLimitSwitch(boolean forward, boolean reverse) {
        // if (forward)
        // motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
        // if (reverse)
        // motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        // LimitSwitchNormal.NormallyOpen);
    }

    public boolean getForwardLimitSwitch() {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public boolean getReverseLimitSwitch() {
        return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }

    public void setBrakeMode(boolean mode) {
        /* Keep a neutral out so we can disable the motor */
        NeutralOut m_brake = new NeutralOut();
        motor.setControl(m_brake);
        if (followId > 0) {
            followMotor.setControl(m_brake);
        }
    }

    public void setPos(double position) {
        // TODO Need to fix this
        // motor.set(ControlMode.Position, position);
    }

    public void setInverted(boolean invert) {
        this.motorInvert = invert;
        StatusCode retval = motor.getConfigurator().refresh(motorConfigs);
        if (retval.isOK()) {
            /* Then set the invert config to the appropriate value */
            motorConfigs.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            retval = motor.getConfigurator().apply(motorConfigs);
        }
        if (followId > 0) {
            retval = followMotor.getConfigurator().refresh(motorConfigs);
            if (retval.isOK()) {
                /* Then set the invert config to the appropriate value */
                motorConfigs.Inverted = invert ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
                retval = followMotor.getConfigurator().apply(motorConfigs);
            }
        }
    }

    public void setRampRate(double timeToFull) {
        //motor.configOpenloopRamp();
        // if (followId > 0) {
        // followMotor.configOpenloopRamp(timeToFull);
        // }
    }

    public double getLastSpeed() {
        return lastSpeed;
    }

    public double getActualSpeed() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getActualVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void periodic() {
        if (!Robot.config.showMotorData)
            return;
        if (count % 50 == 0 && Robot.debug) {
            logPeriodic();
        }
        if (count % 500 == 0)
            updateSmart();
    }

    public void logPeriodic() {
        double pos = getPos();
        if (pos != lastPos) {
            lastPos = pos;
            if (followId > 0) {
                logf("%s motor cur:%.2f temp:%.2f vel:%.2f pos:%.0f inv:%b senP:%b\n", name,
                        motor.getStatorCurrent().getValue(), motor.getDeviceTemp().getValue(),
                        motor.getVelocity().getValue(), pos,
                        motorInvert, sensorPhase);
                logf("%s follow  cur:%.2f temp:%.2f vel:%d pos:%.2f\n", name,
                        followMotor.getStatorCurrent(),
                        followMotor.getDeviceTemp().getValue(), followMotor.getVelocity().getValue(),
                        followMotor.getPosition().getValue());
            } else {
                logf("%s motor cur:%.2f temp:%.2f vel:%d pos:%d inv:%b senP:%b\n", name,
                        motor.getStatorCurrent().getValue(), motor.getDeviceTemp().getValue(),
                        motor.getVelocity().getValue(),
                        pos, motorInvert, sensorPhase);
            }
        }
    }

    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
        /*
         * Peak Current and Duration must be exceeded before current limit is activated.
         * When activated, current will be limited to Continuous Current. Set Peak
         * Current params to 0 if desired behavior is to immediately current-limit.
         */
        // talon.configPeakCurrentLimit(35, 10); /* 35 A */
        // talon.configPeakCurrentDuration(200, 10); /* 200ms */
        // talon.configContinuousCurrentLimit(30, 10); /* 30

        // SupplyCurrentLimitConfiguration(boolean enable, double currentLimit, double
        // triggerThresholdCurrent, double triggerThresholdTime)
        SupplyCurrentLimitConfiguration cl = new SupplyCurrentLimitConfiguration(true, peakAmps, continousAmps,
                durationMilliseconds);
        cl.enable = true;
    }

    public void updateSmart() {
        SmartDashboard.putNumber(name + " Pos", (int) getPos());
        SmartDashboard.putNumber(name + " Cur", round2(motor.getStatorCurrent().getValueAsDouble()));
    }

    public void setSpeed(double speed) {
        if (speed != lastSpeed) {
            motor.set(speed);
            lastSpeed = speed;
        }
    }

    public void forcePercentMode() {
        motor.set(0.001);
    }

    public void setSpeedAbsolute(double speed) {
        motor.set(speed);
        lastSpeed = speed;
    }

    public void stopMotor() {
        motor.set(0);
        lastSpeed = 0;
    }

    public void zeroEncoder() {
        motor.setPosition(0);
    }

    public void setEncoderPosition(double position) {
        motor.setPosition(position);
    }

    public void setPositionPID(PID pid, FeedbackDevice feedBack) {
        // feedBackDevice = feedBack;
        // setPositionPID(0, pid);
        // PIDToMotor(pid, 0, Robot.config.kTimeoutMs);
    }

    public void setVelocityPID(PID pid) {
        PIDToMotor(pid, 1, Robot.config.kTimeoutMs);
    }

    public double getMotorVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public void setVelocity(double velocity) {
        // logf("!!!! Set Velocity for %s to %.0f\n", name, velocity);
        motor.set(velocity);
    }

    public void PIDToMotor(PID pid, int slot, int timeout) {

        // Configure the TalonFX for basic use
        TalonFXConfiguration configs = new TalonFXConfiguration();
        // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and
        // a kV of 2 on slot 0
        configs.Slot0.kP = 1;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 10;
        configs.Slot0.kV = 2;
        // Write these configs to the TalonFX
        motor.getConfigurator().apply(configs);
        logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
    }

    public double getError() {
        return motor.getClosedLoopError().getValue();
    }

    public void logMotorVCS() {
        if (Math.abs(lastSpeed) > .02) {
            logf("%s\n", getMotorVCS(motor));
            if (followId > 0) {
                logf("%s\n", getMotorVCS(followMotor));
            }
        }
    }

    public String getMotorVCS() {
        return getMotorVCS(motor);
    }

    private String getMotorVCS(TalonFX motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getMotorVoltage().getValueAsDouble();
            double outputVoltage = motor.getMotorVoltage().getValueAsDouble();
            double supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
            double statorCurrent = motor.getStatorCurrent().getValueAsDouble();
            return String.format("%s motor volts<%.2f:%.2f> cur<%.2f:%.2f> power<%.2f:%.2f> sp:%.3f", name, bussVoltage,
                    outputVoltage, supplyCurrent, statorCurrent, bussVoltage * supplyCurrent,
                    outputVoltage * statorCurrent, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void setSensorPhase(boolean phase) {
        sensorPhase = phase;
        // Need to code
        // motor.setSensorPhase(phase);
    }

    public void setRampClosedLoop(double rate) {
        // Rate is secondsFromNeutralToFull
        // motor.configClosedloopRamp(rate);
        // Rate is secondsFromNeutralToFull
        // motor.configClosedloopRamp(rate);
    }

    public void setRampOpenLoop(double rate) {
        // Rate is secondsFromNeutralToFull
        // motor.configOpenloopRamp(rate);
    }

    public void setPositionPID(int pidIdx, PID pid) {
        // TODO need to code this
        // Config the sensor used for Primary PID and sensor direction

        /* Config the peak and nominal outputs, 12V means full */
        // motor.configNominalOutputForward(0, Robot.config.kTimeoutMs);
        // motor.configNominalOutputReverse(0, Robot.config.kTimeoutMs);
        // motor.configPeakOutputForward(pid.kMaxOutput, Robot.config.kTimeoutMs);
        // motor.configPeakOutputReverse(pid.kMinOutput, Robot.config.kTimeoutMs);

        // Config the allowable closed-loop error, Closed-Loop output will be neutral
        // within this range. See Table in Section 17.2.1 for native units per rotation.
        // motor.configAllowableClosedloopError(0, pidIdx, Robot.config.kTimeoutMs);
    }
}