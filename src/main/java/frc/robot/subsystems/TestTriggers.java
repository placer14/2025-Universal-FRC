package frc.robot.subsystems;

//import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TestTriggers extends SubsystemBase {
  /** Creates a new ReplaceMeSubs`ystem. */
  DigitalInput sw = new DigitalInput(7);

  public TestTriggers() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Make sure that you declare this subsystem in RobotContainer.java
    if (Robot.count % 5 == 0) {
      SmartDashboard.putBoolean("RIOSW", getSwitch());
    }
  }

  public boolean getSwitch() {
    return !sw.get();
  }
}
