// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://docs.wpilib.org/es/latest/docs/yearly-overview/yearly-changelog.html
// 

package frc.robot;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.YawProvider;
//import dev.doglog.DogLog;
//import dev.doglog.DogLogOptions;

/**
 * 
 * 
 * file:///C:/Users/Public/wpilib/2025/documentation/rtd/frc-docs-latest/index.html#document-docs/software/support/support-resources
 * 
 * software for 2025 --
 * https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html
 * 
 * Sometimes the garbage collector won’t run frequently enough to keep up with
 * the quantity of allocations. As Java provides a way to trigger a garbage
 * collection to occur, running it on a periodic basis may reduce peak memory
 * usage. This can be done by adding a Timer and a periodic check:
 * 
 * Timer m_gcTimer = new Timer();
 * public Robot() {
 * m_gcTimer.start();
 * }
 * public void periodic() {
 * // run the garbage collector every 5 seconds
 * if (m_gcTimer.advanceIfElapsed(5)) {
 * System.gc();
 * }
 * }
 */
public class Robot extends TimedRobot {
  public static int count = 0;
  public static RobotContainer robotContainer;
  public static Optional<Alliance> alliance;
  public static boolean debug = true;
  public static Config config = new Config();
  public static double yaw;
  public static YawProvider yawProvider = new YawProvider();

  @Override
  public void robotInit() {
    //DogLog.setOptions(new DogLogOptions().withLogExtras(false));
    robotContainer = new RobotContainer();
    alliance = DriverStation.getAlliance();
    yaw = yawProvider.getYaw();
    logf("******  Start robot %s with alliance %s yaw:%.2f Battery Volts:%.2f Brownout Volts:%.2f ******\n",
        Config.robotType, alliance.toString(),
        yaw, RobotController.getBatteryVoltage(), RobotController.getBrownoutVoltage());
  }

  @Override
  public void teleopInit() {
    logf("Start Teleop\n");
    //DogLog.log("Start Teleop yaw:", yaw);
    System.gc();
  }

  @Override
  public void robotPeriodic() {
    yaw = yawProvider.getYaw();
    CommandScheduler.getInstance().run();
    if (count % 20 == 4) { // Update Dashboard every 20 cycles or 200 milliseconds (20 ms * 10)
      SmartDashboard.putNumber("Yaw", round2(yaw));
    }
    count++;
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    if (count % 100 == 0) {
      long mem = Runtime.getRuntime().freeMemory();
      SmartDashboard.putNumber("Free Mem", mem);
    }
    if (count % 50 == 0) {
      //System.gc();
    }
    // logf("Count:%d\n", count);
    // DogLog.log("Dog:", count);
  }

  // @Override
  // public void simulationPeriodic() {
  //   if (count % 50 == 0) {
  //     // logf("Count:%d\n", count);
  //     robotContainer.testLeds();
  //   }
  // }
}