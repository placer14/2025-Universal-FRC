package frc.robot;

import static frc.robot.utilities.Util.logf;

import java.io.File;
import java.nio.file.Files;
import java.util.List;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Config {
    public enum RobotType {
        MiniKeith, BlondeMini, Squidward, Kevin, Wooly, Mando, Simulation, DarrylMini, MiniMini, Sibling2025
    };

    // Type of Robot
    public static RobotType robotType = RobotType.MiniKeith;

    // Pneumatic Control Modules Parameters
    public int pcmHubID = -1;

    // Vision Parameters
    public boolean LimeLight = false;
    public boolean ledRing = false;
    public boolean PhotonVision = false;
    public enum DriveType {
        None, Tank, MildTank, AggressiveTank, MildArcade, AggressiveArcade, MildCurvature, CurvatureAggressive
    };

    // Drive Parameters
    public boolean driveTrainJaguar = false;
    public int driveRight = 2;
    public int driveLeft = 3;
    public int driveRightFollow = -1;
    public int driveLeftFollow = -1;
    public static DriveType driveType = DriveType.MildArcade;
    public boolean invertDrivetrain = true;
    public boolean defaultBrakeMode = true;
    public double wheelBase = 0; //
    public double wheelDiameter = 0;
    public double driveTicksPerRevolution = 0;
    public boolean showMotorData = true;
    public double driveTicksPerInch = 0;

    // Climber Parameters
    public boolean climber = false;

    // Miscellaneous Parameter
    public boolean ultraSonicDistance = false;
    public int kTimeoutMs = 30; // default timeout used for messages to the SRX
    public boolean enableCompressor = false;
    public boolean cameraServer = false;
    public boolean joysticksEnabled = true;
    public boolean operatorPadEnabled = false;
    public boolean driverPadEnabled = true;
    public boolean neoPixelsActive = false;
    public double deadZone = 0.085;
    public boolean powerHubToDashBoard = false;
    public boolean ColorSensor = false;
    public PneumaticsModuleType pneumaticType = null;

    // RoboRIO Pin usage
    public final int PWMLedStrip = 9;
    public final int numberOfLeds = 8;
    public final int DIOTestSwitch = 8;
    public final int DIOTestTrigger = 7;

    // Misc parameters
    public boolean frisbeeShooter = false;
    public boolean ballShooter = false;
    public boolean lidar = false;
    public boolean pigeon = false;
    public boolean BNO055Connected = false;
    public boolean PowerDistributionHubV1 = false;
    public boolean PowerDistributionHubV2 = false;
    public boolean PneumaticHUB = false;
    public boolean BlinkTarget = false;
    

    Config() {
        // MiniSRX, Squidward, Kevin, Wooly, Mando
        logf("Start of Robot Config for %s\n", robotType);

    }

    public RobotType getRobotType() {
        return robotType;
    }

    public void getRobotTypeFromFile() {
        String fileName = "/home/lvuser/deploy/robottype.txt";
        List<String> lines = null;
        try {
            lines = Files.readAllLines(new File(fileName).toPath());
        } catch (Exception e) {
            logf("Unable to open file:%d in RoboRio\n", fileName);
        }
        try {
            logf("Robot type from file:%s value:%s/n", fileName, lines.get(0));
            robotType = RobotType.valueOf(lines.get(0));
        } catch (Exception e) {
            logf("Error can't convert %s to valid robot type", lines.get(0));
        }
    }
}