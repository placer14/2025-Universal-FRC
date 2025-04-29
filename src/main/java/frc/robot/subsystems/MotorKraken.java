// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Robot.robotContainer;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class MotorKraken extends SubsystemBase {
  private boolean myLogging = false;
  private boolean testMode = false;
  private String name = "";
  private final TalonFX motor;

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1);

  /* Start at position 0, use slot 2 */
  // private final PositionTorqueCurrentFOC m_positionTorque = new
  // PositionTorqueCurrentFOC(0).withSlot(2);

  /* Keep a brake request so we can disable the motor */
  // private final NeutralOut brake;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public MotorKraken(String name, int id, int followId, boolean logging) {
    this.name = name;
    // Keep a brake request so we can disable the motor
    // brake = new NeutralOut();
    motor = new TalonFX(id);
    configKraken(motor, false);

    /* Make sure we start at 0 */
    motor.setPosition(0);
  }

  private void configKraken(TalonFX motor, boolean velocityTorque) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(12))
        .withPeakReverseVoltage(Volts.of(-12));

    configs.Slot2.kP = 60; // An error of 1 rotation results in 60 A output
    configs.Slot2.kI = 0; // No output for integrated error
    configs.Slot2.kD = 6; // A velocity of 1 rps results in 6 A output
    // Peak output of 120 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(120))
        .withPeakReverseTorqueCurrent(Amps.of(-120));

    /*
     * Voltage-based velocity requires a velocity feed forward to account for the
     * back-emf of the motor
     */
    configs.Slot1.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot1.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / rotation per second
    configs.Slot1.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative

    if (velocityTorque) {
      /*
       * Torque-based velocity does not require a velocity feed forward, as torque
       * will accelerate the rotor up to the desired velocity by itself
       */
      configs.Slot2.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
      configs.Slot2.kP = 5; // An error of 1 rotation per second results in 5 A output
      configs.Slot2.kI = 0; // No output for integrated error
      configs.Slot2.kD = 0; // No output for error derivative
      // Peak output of 40 A
      configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
          .withPeakReverseTorqueCurrent(Amps.of(-40));
    }

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

  private void setSpeed(double value) {
    motor.set(value);
  }

  private void setPos(double value) {
    motor.setControl(positionVoltage.withPosition(value));
  }

  private void setVelocity(double value) {
    /* Use velocity voltage */
    motor.setControl(velocityVoltage.withVelocity(value));
  }

  @Override
  public void periodic() {
    // double desiredRotations = m_joystick.getLeftY() * 10; // Go for plus/minus 10
    // rotations
    // if (Math.abs(desiredRotations) <= 0.1) { // Joystick deadzone
    // desiredRotations = 0.0;
    // }

    // if (m_joystick.getLeftBumperButton()) {
    // /* Use position voltage */
    // motor.setControl(m_positionVoltage.withPosition(desiredRotations));
    // } else if (m_joystick.getRightBumperButton()) {
    // /* Use position torque */
    // motor.setControl(m_positionTorque.withPosition(desiredRotations));
    // } else {
    // /* Disable the motor instead */
    // motor.setControl(m_brake);
    // }

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
      logf("%s velocity:%.2f RPM:%.2f current:%.2f pos:%.2f err:%.2f\n", name, velocity, velocity* 60.0, current, pos, err);
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
        value = driveController.getHID().getPOV() / (270.0/100.0);
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