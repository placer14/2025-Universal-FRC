package frc.robot;

//import static frc.robot.Robot.yaw;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config.RobotType;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.MotorConfigs;
import frc.robot.subsystems.MotorFlex;
import frc.robot.subsystems.MotorSRX;
import frc.robot.subsystems.NeoMotor;
import frc.robot.subsystems.TestMiniMotors;
import frc.robot.subsystems.TestBlondeMotors;
import frc.robot.subsystems.TestTriggers;
import frc.robot.subsystems.MotorConfigs.MotorTypes;

/**
 * This class is where the bulk of the robot should be declared. Since
 * be declared. Since Command-based is a
 * "declarative" paradigm, very little robot
 * logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).
 * Instead, the structure ofthe robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final static CommandXboxController driveController = new CommandXboxController(2);
    // private final static CommandXboxController operatorController = new
    // CommandXboxController(3);
    // private final static XboxController operatorHID =
    // operatorController.getHID();
    private final static XboxController driveHID = driveController.getHID();

    private static LedSubsystem leds = new LedSubsystem();
    public DrivetrainSRX drivetrainSRX;
    public TestMiniMotors testMiniMotors;
    public TestBlondeMotors testblondeMotors;

    private TestTriggers triggers = new TestTriggers();
    private double setSpeedFromTrigger(){
        double leftValue = driveController.getLeftTriggerAxis();
        double rightValue = driveController.getRightTriggerAxis();
        if (leftValue > 0.05){
            return leftValue;
        }
        if (rightValue > 0.05){
            return -rightValue;
        }
        return 0.0; 
    }
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set the default Robot Mode to Cube
        switch (Config.robotType) {
            case Simulation:
                break;
            case BlondeMini:
                new DrivetrainSRX(driveHID);
                new NeoMotor(driveHID);
                // new TestBlondeMotors(driveHID, leds);
                // new TestSlider(driveHID, leds);
                break;
            case DarrylMini:
                new DrivetrainSRX(driveHID);
                MotorSRX dmotor = new MotorSRX("DarrylSRX", 10, -1, true);
                Command darrylMoveBack = Commands.run(() -> dmotor.setSpeed(setSpeedFromTrigger()), dmotor);
                driveController.start().onTrue(darrylMoveBack); 
                break;
            case MiniMini:
                MotorSRX mmmotor = new MotorSRX("MiniSRX", 10, -1, true);
                Command miniMove = Commands.run(() -> mmmotor.setSpeed(driveController.getLeftTriggerAxis()), mmmotor);
                driveController.start().onTrue(miniMove);
                break;
            case MiniSRX: // Test mini
                // Use Talon SRX for drive train
                drivetrainSRX = new DrivetrainSRX(driveHID);
                // Setup to test Flex Motor
                MotorConfigs conf = new MotorConfigs(MotorTypes.SparkFlexMaxMotion);
                MotorFlex motor = new MotorFlex("TestFlex", 10, -1, false, false, conf);
                motor.setLogging(true);
                Command l = Commands.run(() -> motor.setSpeed(driveController.getLeftTriggerAxis()), motor);
                driveController.start().onTrue(l);
            case Squidward:
                // Uses Talon SRX for drive train())
            case Kevin: // Ginger Bread Robot
                // Uses Talon SRX for drive train
                break;
            case Wooly: // Big ball shooter
                // Uses Jaguars for drive train and shooter
                // Uses PCM to control shooter tilt and shooter activate
                break;

            case Mando: // Train engine
                // Use SparkMax motors for drive train
                break;
        }
        logf("FInished Creating RobotContainer\n");
        if (Config.robotType != RobotType.Simulation) {
            configureButtonBindings();
        }
    }

    // private Command controlFlex(MotorDef motor) {
    // return Commands.run(() ->
    // motor.setSpeed(driveController.getLeftTriggerAxis()));
    // }

    public void testLeds() {
        leds.setColors(0, 127, 0);
    }

    public double squareWithSign(double v) {
        return (v < 0) ? -(v * v) : (v * v);
    }

    // The following code is an attempt to learn how to program commands

    Command zy = new InstantCommand(new Runnable() {
        public void run() {
            Robot.yawProvider.zeroYaw();
            // testMotors.resetPos();
        }
    });

    Command testCmd = new InstantCommand(new Runnable() {
        public void run() {
            logf("Hit x on Game Pad\n");
        }
    });

    Command testTrigger = new InstantCommand(new Runnable() {
        public void run() {
            logf("Switch Hit\n");
        }
    });

    Trigger tr = new Trigger(triggers::getSwitch);

    private void configureButtonBindings() {
        // operatorHID.getAButton(); // Avoids a not used problem
        driveController.x().onTrue(testCmd);
        driveController.back().whileTrue(new InstantCommand(new Runnable() {
            public void run() {
                Robot.yawProvider.zeroYaw();
                logf("Hit back on Game Pad\n");
            }
        }));
        // driveController.start().onTrue(zy);
        tr.onTrue(testTrigger);
    }

    // Initializes a DigitalInput
    DigitalInput input = new DigitalInput(8);
    // Creates a Debouncer in "both" mode.
    Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    void deB() {
        // If false the signal must go true for at least .1 seconds before read
        if (m_debouncer.calculate(input.get())) {
            logf("Input Changed:%b\n", input.get());
        }
    }
}
