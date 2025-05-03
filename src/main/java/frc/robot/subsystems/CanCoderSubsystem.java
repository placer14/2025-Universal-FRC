package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CanCoderSubsystem extends SubsystemBase {

  private String name = "";
  private boolean showOnSmart = false;
  private int id = 0;
  private CANcoder canCoder;

  public CanCoderSubsystem(String name, int id, boolean showOnSmart) {
    this.name = name;
    this.id = id;
    this.showOnSmart = showOnSmart;
    logf("Start of Cancoder Subsystem id:%d showOnSmart %b\n", this.id, showOnSmart);
    canCoder = new CANcoder(id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Make sure that you declare this subsystem in RobotContainer.java
    if (showOnSmart) {
      SmartDashboard.putNumber(name, canCoder.getAbsolutePosition().getValueAsDouble());
    }
  }

  public double getAngle() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }
}