package frc.robot.subsystems;

import static frc.robot.Robot.count;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;
import static frc.robot.Robot.robotContainer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// setSensorPhase() should change the direction reported in the getSelectedSensor*() methods
// (but not the SensorCollection methods).
// It should also change the direction reported for "PID0" in a self-test snapshot,
// but not the position reported by "Quad/MagEnc(rel)"

public class MotorSRX extends SubsystemBase implements MotorDef {
  private TalonSRX motor;
  private TalonSRX followMotor;
  private String name;
  private int id;
  private int followId;
  private double lastSpeed = 0;
  private double lastPos = 0;
  private boolean myLogging = false;
  private ErrorCode errorCode;
  private FeedbackDevice feedBackDevice = FeedbackDevice.QuadEncoder;
  private int numberCyclesForDisplay = 1000000;
  private boolean enableTestMode = false;

  public MotorSRX(String name, int id, int followId, boolean logging) {
    this.name = name;
    this.id = id;
    this.followId = followId;
    myLogging = logging;
    motor = new TalonSRX(this.id);
    errorCode = motor.configFactoryDefault();
    if (errorCode != ErrorCode.OK) {
      logf("????????? Motor %s Error: %s ??????????\n", name, errorCode);
    }
    if (followId > 0) {
      followMotor = new TalonSRX(followId);
      errorCode = followMotor.configFactoryDefault();
      followMotor.follow(motor);
      if (errorCode != ErrorCode.OK) {
        logf("????????? Follow Motor %s Error: %s ??????????\n", name, errorCode);
        followId = -followId;
      }
    }
    motor.setSensorPhase(true);
    motor.getSensorCollection().setQuadraturePosition(0, 0);
    if (followId > 0)
      logf(
          "Created %s motor ids:<%d,%d> firmware:<%d,%d> voltage:<%.1f,%.1f>\n",
          name, id, followId, motor.getFirmwareVersion(), followMotor.getFirmwareVersion(), motor.getBusVoltage(),
          followMotor.getBusVoltage());
    else
      logf(
          "Created %s motor id:%d firmware:%d voltage:%.1f\n",
          name, id, motor.getFirmwareVersion(), motor.getBusVoltage());
  }

  public void setLogging(boolean value) {
    myLogging = value;
  }

  public void setSmartTicks(int numberLoopsForDisplay) {
    this.numberCyclesForDisplay = numberLoopsForDisplay;
  }

  public void setTestMode(boolean value) {
    logf("Set Flex Test Mode:%b\n", value);
    enableTestMode = value;
  }

  public double getPos() {
    return motor.getSelectedSensorPosition();
  }

  public void enableLimitSwitch(boolean forward, boolean reverse) {
    if (forward)
      motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    else
      motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
    if (reverse)
      motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    else
      motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
  }

  public boolean getForwardLimitSwitch() {
    return motor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getReverseLimitSwitch() {
    return motor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void setBrakeMode(boolean mode) {
    motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    if (followId > 0)
      followMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
  }

  public void setPos(double position) {
    lastPos = position;
    motor.selectProfileSlot(0, 0);
    motor.set(ControlMode.Position, position);
  }

  public void setPosMM(double position) {
    motor.set(ControlMode.MotionMagic, position);
  }

  public void setVelocity(double velocity) {
    // logf("!!!! Set Velocity for %s to %.0f\n", name, velocity);
    motor.selectProfileSlot(1, 0);
    motor.set(ControlMode.Velocity, velocity);
  }

  public void setInverted(boolean invert) {
    motor.setInverted(invert);
    if (followId > 0) {
      followMotor.setInverted(invert);
    }
  }

  public double getLastSpeed() {
    return lastSpeed;
  }

  public double getActualVelocity() {
    return motor.getSelectedSensorVelocity(0);
  }

  public int getAnalogPos() {
    return motor.getSensorCollection().getAnalogIn() + 4096 - 1078;
  }

  public void periodic() {
    if (count % 50 == 0 && Robot.debug) {
      logPeriodic();
    }
    if (count % numberCyclesForDisplay == 0)
      updateSmart();

    if (enableTestMode)
      testCases(RobotContainer.leds);
  }

  public void logPeriodic() {
    double pos = motor.getSensorCollection().getQuadraturePosition();
    if (pos != lastPos) {
      lastPos = pos;
      if (myLogging) {
        logf("%s\n", getMotorVCS());
        if (followId > 0) {
          logf("%s\n", getMotorVCS(followMotor));
        }
      }
    }
  }

  public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
    /*
     * Peak Current and Duration must be exceeded before current limit is activated.
     * When activated, current will be limited to Continuous Current. Set Peak
     * Current params to 0 if desired behavior is to immediately current-limit.
     */
    motor.configPeakCurrentLimit(peakAmps, Robot.config.kTimeoutMs);
    motor.configPeakCurrentDuration(durationMilliseconds, Robot.config.kTimeoutMs);
    motor.configContinuousCurrentLimit(continousAmps, Robot.config.kTimeoutMs);
    motor.enableCurrentLimit(true); // Honor initial setting
  }

  public void updateSmart() {
    SmartDashboard.putString("Mode", mode.toString());
    SmartDashboard.putNumber("Pos", (int) motor.getSensorCollection().getQuadraturePosition());
    SmartDashboard.putNumber("Cur", round2(getMotorCurrent()));
    SmartDashboard.putNumber("Vel", round2(getActualVelocity()));
    SmartDashboard.putNumber("Err", getError());
    SmartDashboard.putBoolean("forL", getForwardLimitSwitch());
    SmartDashboard.putBoolean("revL", getReverseLimitSwitch());
  }

  public void setSpeed(double speed) {
    if (speed == 0.0) {
      motor.set(ControlMode.PercentOutput, 0);
    }
    if (speed != lastSpeed) {
      motor.set(ControlMode.PercentOutput, speed);
    }
    lastSpeed = speed;
  }

  public void setSpeedAbsolute(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
    lastSpeed = speed;
  }

  public void stopMotor() {
    motor.set(ControlMode.PercentOutput, 0);
    lastSpeed = 0;
  }

  public void zeroEncoder() {
    motor.setSelectedSensorPosition(0);
  }

  public void setEncoderPosition(double position) {
    motor.getSensorCollection().setQuadraturePosition((int) position, Robot.config.kTimeoutMs);
  }

  public void forcePercentMode() {
    motor.set(ControlMode.PercentOutput, 0.001);
  }

  public void setPositionPID(PID pid, int slot, FeedbackDevice feedBack) {
    feedBackDevice = feedBack;
    setPositionPID(slot, pid);
    PIDToMotor(pid, slot, Robot.config.kTimeoutMs);
  }

  public void setVelocityPID(PID pid, int slot, FeedbackDevice feedBack) { // tod at some point fix this name
    feedBackDevice = feedBack;
    PIDToMotor(pid, slot, Robot.config.kTimeoutMs);
  }

  public void setMotionMagicPID(PID pid, int slot, FeedbackDevice feedBack) { // tod at some point fix this name
    feedBackDevice = feedBack;
    PIDToMotor(pid, slot, Robot.config.kTimeoutMs);
  }

  public double getMotorVoltage() {
    return motor.getMotorOutputVoltage();
  }

  public double getBatteryVoltage() {
    return motor.getBusVoltage();
  }

  public void PIDToMotor(PID pid, int slot, int timeout) {
    motor.config_kP(slot, pid.kP, timeout);
    motor.config_kI(slot, pid.kI, timeout);
    motor.config_kD(slot, pid.kD, timeout);
    motor.config_kF(slot, pid.kF, timeout);
    motor.configPeakOutputForward(pid.kMaxOutput);
    motor.configPeakOutputReverse(pid.kMinOutput);
    motor.config_IntegralZone(slot, (int) pid.kIz, timeout);
    motor.configAllowableClosedloopError(slot, pid.allowableCloseLoopError, timeout);
    motor.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
    if (pid.kV > 0) {
      // Set parameters for motion magic
      motor.configMotionCruiseVelocity(pid.kV);
      motor.configMotionAcceleration(pid.kA);
      motor.configAllSettings(null);
    }
    logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
  }

  public double getError() {
    return motor.getClosedLoopError(0);
  }

  public void logMotorVCS() {
    if (Math.abs(lastSpeed) > .02) {
      logf("%s\n", getMotorVCS());
      if (followId > 0) {
        logf("%s\n", getMotorVCS(followMotor));
      }
    }
  }

  public String getMotorVCS() {
    return getMotorVCS(motor);
  }

  private String getMotorVCS(TalonSRX motor) {
    double bussVoltage = motor.getBusVoltage();
    double outputVoltage = motor.getMotorOutputVoltage();
    double supplyCurrent = motor.getSupplyCurrent();
    double statorCurrent = motor.getStatorCurrent();
    return String.format(
        "%s motor volts<%.2f:%.2f> cur<%.2f:%.2f> power<%.2f:%.2f> last sp:%.3f",
        name, bussVoltage, outputVoltage, supplyCurrent, statorCurrent, bussVoltage * supplyCurrent,
        outputVoltage * statorCurrent, lastSpeed);
  }

  public double getMotorCurrent() {
    return motor.getStatorCurrent();
  }

  public double getFollowCurrent() {
    if (followMotor == null) {
      return 0;
    }
    return followMotor.getStatorCurrent();
  }

  public void setSensorPhase(boolean phase) {
    motor.setSensorPhase(phase);
  }

  public void setPositionPID(int pidIdx, PID pid) {
    // Config the sensor used for Primary PID and sensor direction
    motor.configSelectedFeedbackSensor(feedBackDevice, pidIdx, Robot.config.kTimeoutMs);

    // Ensure sensor is positive when output is positive
    // motor.setSensorPhase(sensorPhase);

    // Set based on what direction you want forward/positive to be.
    // This does not affect sensor phase.
    // motor.setInverted(motorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    motor.configNominalOutputForward(0, Robot.config.kTimeoutMs);
    motor.configNominalOutputReverse(0, Robot.config.kTimeoutMs);
    motor.configPeakOutputForward(pid.kMaxOutput, Robot.config.kTimeoutMs);
    motor.configPeakOutputReverse(pid.kMinOutput, Robot.config.kTimeoutMs);

    // Config the allowable closed-loop error, Closed-Loop output will be neutral
    // within this range. See Table in Section 17.2.1 for native units per rotation.
    int positionErr = 1;
    motor.configAllowableClosedloopError(pidIdx, positionErr, Robot.config.kTimeoutMs);
  }

  public void setRampClosedLoop(double rate) {
    // Rate is secondsFromNeutralToFull
    motor.configClosedloopRamp(rate);
  }

  public void setRampOpenLoop(double rate) {
    // Rate is secondsFromNeutralToFull
    motor.configOpenloopRamp(rate);
  }

  // Example settings for motion magic
  // slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
  // slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  // slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  // slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
  // slot0.kI = 0; // No output for integrated error
  // slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

  // Setting the Feed Forward for a velcity PID
  // Using Tuner (Self-test Snapshot or Plotter), we’ve measured a peak velocity of 9326 native units per 100ms at 100% output. 
  // Get peak velocity - this can also be retrieved using getSelectedSensorVelocity (routine or VI).
  // They measured a peak velocity of 9326 native units per 100ms at 100% output. 
  // However, many mechanical systems and motors are not perfectly linear (though they are close). 
  // To account for this, we should calculate our feed forward using a measured velocity around the percent output we will usually run the motor.
  // For our mechanism, we will typically be running the motor ~75% output. 
  // We then use Tuner (Self-test Snapshot or Plotter) to measure our velocity - in this case, we measure a velocity of 7112 native units per 100ms.
  // Now let’s calculate a Feed-forward gain so that 75% motor output is calculated when the requested speed is 7112 native units per 100ms.
  // F-gain = (75% X 1023) / 7112 F-gain = 0.1079
  // Let’s check our math, if the target speed is 7112 native units per 100ms, 
  // Closed-loop output will be (0.1079 X 7112) => 767.38 (75% of full forward).


  public void setupForTestCasesRedMotor(boolean enableTest, LedSubsystem leds) {
    PID positionPID = new PID("Pos", 0.08, 0, 0, 0, 0, -1, 1, false);
    PID velocityPID = new PID("Vel", .005, 0, 0, 0, 1.5, -1, +1, false);
    PID motionMagicPID = new PID("MM", 0.08, 0, 0, 0, 0, -1, 1, true);
    setUpForTestCases(positionPID, velocityPID, motionMagicPID, enableTest, leds);
  }

  public void setupForTestCasesGrayMotor(boolean enableTest, LedSubsystem leds) {
    PID positionPID = new PID("Pos", 0.08, 0, 0, 0, 0, -1, 1, true);
    PID velocityPID = new PID("Vel", .005, 0, 0, 0, 1.5, -1, +1, true);
    PID motionMagicPID = new PID("MM", 0.08, 0, 0, 0, 0, -1, 1, true);
    setUpForTestCases(positionPID, velocityPID, motionMagicPID, enableTest, leds);
  }
  public void setUpForTestCases(PID positionPID, PID velocityPID, PID motionMagicPID, boolean enableTest, LedSubsystem leds) {
    logf("Start of Test SRX Subsystem\n");
    setInverted(true);
    setSensorPhase(true);
    enableLimitSwitch(false, false);
    //positionPID.showPID();
    //velocityPID.showPID();
    //motionMagicPID.showPID();
    // Motion Magic messes things up positionPID.setMotionMagicSRX(.5, 2.0);
    setPositionPID(positionPID, 0, FeedbackDevice.QuadEncoder); // set pid for SRX
    setVelocityPID(velocityPID, 1, FeedbackDevice.QuadEncoder);
    setMotionMagicPID(motionMagicPID, 2, FeedbackDevice.QuadEncoder);
    setEncoderPosition(0);
    setSmartTicks(5); // Set to update SmartDashBoard every 100 milli seconds
    enableTestMode = enableTest;
    leds.setRangeOfColor(0, mode.ordinal(), 0, 127, 0);
  }

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
  double setP = 0;

  void testCases(LedSubsystem leds) {
    CommandXboxController driveController = RobotContainer.driveController;
    double value = 0.0;
    // Hiting the start button moves to the next control method
    boolean start = driveController.start().getAsBoolean();
    if (start && !lastStart) {
      setSpeed(0.0);
      setEncoderPosition(0);
      mode = mode.next(); // Get the next mode
      logf("New Test Mode:%s Ordinal:%s\n", mode, mode.ordinal());
    }
    lastStart = start;
    switch (mode) {
      case POSITION:
        value = driveController.getHID().getPOV() * 200;
        if (value >= 0.0 && setP != value) {
          setPos(value);
          setP = value;
          logf("SRX set position:%.2f\n", value);
        }
        break;
      case VELOCITY:
        value = driveController.getHID().getPOV() * 1.0;
        if (value >= 0) {
          setVelocity(value);
          logf("SRX set velocity:%.2f\n", value);
          setP = value;
        }
        break;
      case MOTIONMAGIC:
        setP = 0.0;
        break;
      case SPEED:
        value = robotContainer.getSpeedFromTriggers();
        if (Math.abs(value) > 0.05)
          logf("Set Test speed:%.2f\n", value);
        setSpeed(value);
        setP = value;
        break;
    }
    leds.setAllColors(0, 0, 0);
    leds.setRangeOfColor(0, mode.ordinal(), 0, 127, 0);
    SmartDashboard.putNumber("SetP", setP);
  }
}
