package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;

public class MotorConfigs {
    public String name = "";
    @SuppressWarnings("removal")
    public ControlType controlType = ControlType.kSmartMotion;
    public int id = 0;
    public int followID = -1;
    public int currentLimit = 40;
    public boolean logging = false;
    public boolean inverted = false;
    public boolean brakeMode = true;
    public boolean forwardLimit = true;
    public boolean reverseLimit = true;
    public double openLoopRampRate = 1;
    public double closedLoopRampRate = 1;
    public double positionConversionFactor = 1;
    public double velocityConversionFactor = 1;
    public int maxVelocity = 2000;
    public int maxAcceration = 1500;
    public double maxOutput = 1;
    public double kP = 0.00005;
    public double kI = 0.000001;
    public double kD = 0;
    public double kFF = .000156;

    public enum MotorTypes {
        SparkFlexSmartMotion, SparkFlexMaxMotion, 
    }

    MotorConfigs(MotorTypes type) {
        switch (type) {
            case SparkFlexMaxMotion:
                kP = .14;
                break;
            case SparkFlexSmartMotion:
                break;

        }
    }

}