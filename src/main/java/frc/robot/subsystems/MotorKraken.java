package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Robot.robotContainer;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// Motion Magic for Kraken has some code at
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/MotionMagic/src/main/java/frc/robot/Robot.java

public class MotorKraken extends SubsystemBase {
  private boolean myLogging = false;
  private boolean testMode = false;
  private String name = "";
  private final TalonFX motor;
  // Start at position 0, use slot 0 for Postion Control
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  // Start at velocity 0, use slot 1 for Velocity Control
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1);
  // Start at position 0, use slot 2 for position motion magic
  private final MotionMagicVoltage smartPositionVoltage = new MotionMagicVoltage(0).withSlot(2);

  public MotorKraken(String name, int id, int followId, boolean logging) {
    this.name = name;
    // Keep a brake request so we can disable the motor
    // brake = new NeutralOut();
    motor = new TalonFX(id);
    configKraken(motor);
    /* Make sure we start the encoder at positon 0 */
    motor.setPosition(0);
  }

  private void configKraken(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Set slot 0 for position PID
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(12))
        .withPeakReverseVoltage(Volts.of(-12));

    // * Voltage-based velocity requires a velocity feed forward
    configs.Slot1.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot1.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / rotation per second
    configs.Slot1.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative

    /* Configure gear ratio */
    FeedbackConfigs fdb = configs.Feedback;
    fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    // Configure Motion Magic 5 (mechanism) rotations per second cruise
    configs.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5));
    // Take approximately 0.5 seconds to reach max vel
    configs.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10));
    // Take approximately 0.1 seconds to reach max accel
    configs.MotionMagic.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    configs.Slot2.kS = 0.25; // Add 0.25 V output to overcome static friction
    configs.Slot2.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    configs.Slot2.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    configs.Slot2.kP = 60; // A position error of 0.2 rotations results in 12 V output
    configs.Slot2.kI = 0; // No output for integrated error
    configs.Slot2.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
    // configs.MotionMagic.withMotionMagicCruiseVelocity(10.0); // Reduced from 20.0
    // configs.MotionMagic.withMotionMagicAcceleration(20.0); // Reduced from 40.0
    // configs.MotionMagic.withMotionMagicJerk(2000.0); // Reduced from 4000.0

    StatusCode status = motor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      logf("Unable to set status for Kraken motor:%s\n", name);
      return;
    }
  }

  public void setLogging(boolean value) {
    myLogging = value;
  }

  public void setTestMode(boolean value) {
    testMode = value;
  }

  // Set speed as a value from -1 to 1
  public void setSpeed(double value) {
    motor.set(value);
  }

  // Set the position of the motor to a value
  private void setPos(double value) {
    motor.setControl(positionVoltage.withPosition(value));
  }

  // Set the postion of the motor to a value using Magic Motion
  private void setPositionMotionMagic(double value) {
    motor.setControl(smartPositionVoltage.withPosition(value).withSlot(2));
  }

  private void setVelocity(double value) {
    /* Use velocity voltage */
    motor.setControl(velocityVoltage.withVelocity(value));
  }

  @Override
  public void periodic() {
    double err = motor.getClosedLoopError().getValueAsDouble();
    double pos = motor.getPosition().getValueAsDouble();
    double velocity = motor.getVelocity().getValueAsDouble();
    double current = motor.getStatorCurrent().getValueAsDouble();
    SmartDashboard.putNumber(name + " Err", err);
    SmartDashboard.putNumber(name + " Enc", pos);
    SmartDashboard.putNumber(name + " Vel", velocity);
    SmartDashboard.putNumber(name + " RPM", velocity * 60.0); // Get Velocity in PRM
    SmartDashboard.putNumber(name + " Cur", current);
    SmartDashboard.putString("Mode:", mode.toString());
    if (Math.abs(velocity) > 0.05 && myLogging && Robot.count % 10 == 0) {
      logf("%s velocity:%.2f RPM:%.2f current:%.2f pos:%.2f err:%.2f\n", name, velocity, velocity * 60.0, current, pos,
          err);
    }
    if (testMode)
      testCases();
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
  double lastValue = 0.0;

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
          SmartDashboard.putNumber(name + " SetP", value);
          logf("%s set position:%.2f\n", name, value);
        }
        break;
      case VELOCITY:
        // POV 270 degrees is 100
        value = driveController.getHID().getPOV() / (270.0 / 100.0);
        if (lastValue != value) {
          lastValue = value;
          if (value >= 0) {
            SmartDashboard.putNumber(name + " SetP", value);
            setVelocity(value);
            logf("%s set velocity:%.2f\n", name, value);
          }
        }
        break;
      case MOTIONMAGIC:
        value = driveController.getHID().getPOV() / 10.0;
        if (lastValue != value) {
          lastValue = value;
          if (value >= 0) {
            SmartDashboard.putNumber(name + " SetP", value);
            setPositionMotionMagic(value);
            logf("%s set magic mition position:%.2f\n", name, value);
          }
        }
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