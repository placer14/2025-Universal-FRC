// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MotorConfigs.MotorTypes;
//import frc.robot.subsystems.MotorDef;

public class TestBlondeMotors extends SubsystemBase {

    private MotorSparkMax motorMax;
    private NeoMotor neoMotor;
    private XboxController driveHID;
    private int lastPov = -1;
    private MotorConfigs motorC = new MotorConfigs(MotorTypes.SparkFlexSmartMotion);
    private LedSubsystem leds;

    public TestBlondeMotors(XboxController driveHID, LedSubsystem leds) {
        neoMotor = new NeoMotor(driveHID); 
        this.driveHID = driveHID;
        this.leds = leds;
        logf("Start of Test Motors Subsystem for blonde Mini\n");
       // motorMax = new MotorSparkMax("Max", 1, -1, true, false, motorC);
    }

    public void resetPos() {
        motorMax.setEncoderPosition(0);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        // Make sure that you declare this subsystem in RobotContainer.java
        int pov = driveHID.getPOV();
        if (pov == -1) {
            double axisL = driveHID.getLeftTriggerAxis();
            double axisR = driveHID.getRightTriggerAxis();
            if (driveHID.getLeftBumperButtonPressed()) {
                axisL *= -1;
            }
            if (driveHID.getRightBumperButtonPressed()) {
                axisR *= -1;
            }
            lastPov = -1;
            if (Math.abs(axisL) > 0) {
                SmartDashboard.putNumber("FlexSP", axisL);
                motorMax.setSpeed(axisL);
                leds.setColors(127, 0, 0);
                return;
            } else {
                motorMax.setSpeed(0);
            }
            if (Math.abs(axisR) > 0) {
                SmartDashboard.putNumber("SRXSP", axisR);
                leds.setColors(127, 0, 0);
                return;
            } else {
            }
            leds.setColors(0, 127, 0);
            return;
        } else {
            if (pov != lastPov) {
                long begin = RobotController.getFPGATime();
                //logf("TestMotors setEncoder %.2f\n", pov / 90.0);
                motorMax.setPos(pov / 90);
                lastPov = pov;
                SmartDashboard.putNumber("SetPTO", (RobotController.getFPGATime() - begin) / 1000);
            }
        }
    }
}