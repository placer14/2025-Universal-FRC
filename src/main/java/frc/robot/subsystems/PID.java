package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PID extends SubsystemBase {

  // Initial (default) PID coefficients
  public double kP = 0.5;
  public double kI = 0;
  public double kD = 0;
  public double kIz = 0;
  public double kF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double kA = 0;
  public double kV = 0;
  private boolean smart = false;

  public double maxIntegralAccumulation;
  public double resetIntegralAccumulationThreshold;
  public int allowableCloseLoopError;
  public boolean PIDChanged = false;
  public String name;

  // Set PID using the default PID
  public PID(String name, boolean smart) {
    this.name = name;
    this.smart = smart;
  }

  // Set PID using the unique values for P, I, D the remaining values are default
  public PID(String name, double kP, double kI, double kD, boolean smart) {
    this.name = name;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.name = name;
    this.smart = smart;
  }

  // Set PID using the unique values for all parameters
  public PID(
      String name,
      double kP,
      double kI,
      double kD,
      double kIz,
      double kFF,
      double min,
      double max,
      boolean smart) {
    this.name = name;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kIz = kIz;
    this.kF = kFF;
    this.kMaxOutput = max;
    this.kMinOutput = min;
    this.name = name;
    this.smart = smart;
  }

  public PID(
      String name,
      double kP,
      double kI,
      double kD,
      double kIz,
      double kFF,
      double maxIntegralAccumulation,
      double resetIntegralAccumulationThreshold,
      int allowableCloseLoopError,
      boolean smart) {
    this.name = name;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kIz = kIz;
    this.kF = kFF;
    this.smart = smart;
    this.name = name;
    logf("Create %s pid %s \n", name, getPidData());
  }

  @Override
  public void periodic() {
    SmartDashboard.getEntry("Button");
    // SmartDashboard.getEntry("Button");
    // smart = false; // todo fix at some point
    if (smart) {
      PIDChanged = updatePID();
      if (PIDChanged) {
        showPID();
      }
    }
  }

  public String getPidData() {
    return String.format(
        "P:%f I:%f D:%f IZ:%f FF:%f Min:%f Max:%f", kP, kI, kD, kIz, kF, kMinOutput, kMaxOutput);
  }

  public void setMotionMagicSRX(double kMaxVelocity, double kMaxAcceleration) {
    this.kMinOutput = kMaxVelocity;
    this.kA = kMaxAcceleration;
  }

  public void setMinMax(double min, double max) {
    kMaxOutput = max;
    kMinOutput = min;
    if (smart) {
      SmartDashboard.putNumber(name + " Mx", kMaxOutput);
      SmartDashboard.putNumber(name + " Mi", kMinOutput);
    }
  }

  public void showPID() {
    // display PID coefficients on SmartDashboard
    if (smart) {
      //SmartDashboard.putString("PID Name", name);
      SmartDashboard.putNumber(name + " P", kP);
      SmartDashboard.putNumber(name + " I", kI);
      SmartDashboard.putNumber(name + " D", kD);
      SmartDashboard.putNumber(name + " IZ", kIz);
      SmartDashboard.putNumber(name + " F", kF);
      SmartDashboard.putNumber(name + " Mx", kMaxOutput);
      SmartDashboard.putNumber(name + " Mi", kMinOutput);
      // SmartDashboard.putNumber("Set Rotations", 0);
    }
  }

  private boolean updatePID() {
    boolean change = false;
    if (smart) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber(name + " P", 0);
      double i = SmartDashboard.getNumber(name + " I", 0);
      double d = SmartDashboard.getNumber(name + " D", 0);
      double iz = SmartDashboard.getNumber(name + " IZ", 0);
      double ff = SmartDashboard.getNumber(name + " F", 0);
      double max = SmartDashboard.getNumber(name + " Mx", 0);
      double min = SmartDashboard.getNumber(name + " Mi", 0);
      if ((p != kP)) {
        logf("PID old p:%.2f new p:%.2f\n", kP, p);
        kP = p;
        change = true;
      }
      if ((i != kI)) {
        kI = i;
        change = true;
      }
      if ((d != kD)) {
        kD = d;
        change = true;
      }
      if ((iz != kIz)) {
        kIz = iz;
        change = true;
      }
      if ((ff != kF)) {
        kF = ff;
        change = true;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        kMinOutput = min;
        kMaxOutput = max;
        change = true;
      }
      if (change) {
        logf("PID %s Changed %s\n", name, getPidData());
      }
    }
    return change;
  }
}
