package frc.robot;

//import static frc.robot.Robot.yaw;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config.RobotType;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.NeoMotor;
import frc.robot.subsystems.TestMiniMotors;
import frc.robot.subsystems.TestBlondeMotors;
//import frc.robot.subsystems.YawProvider;
import frc.robot.subsystems.TestTriggers;

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
    private final static CommandXboxController driveController = new CommandXboxController(2);
    //private final static CommandXboxController operatorController = new CommandXboxController(3);
    //private final static XboxController operatorHID = operatorController.getHID();
    private final static XboxController driveHID = driveController.getHID();

    private static LedSubsystem leds = new LedSubsystem();
    public DrivetrainSRX drivetrainSRX;
    public TestMiniMotors testMiniMotors;
    public TestBlondeMotors testblondeMotors;

    private TestTriggers triggers = new TestTriggers();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set the default Robot Mode to Cube
        switch (Config.robotType) {
            case Simulation:
                break;
            case blondeMini:
                 new DrivetrainSRX(driveHID);
                 new NeoMotor(driveHID);
                 //new TestBlondeMotors(driveHID, leds);
                //new TestSlider(driveHID, leds);
                break;
            case MiniSRX: // Test mini
                // Use Talon SRX for drive train
                drivetrainSRX = new DrivetrainSRX(driveHID);
                // MotorDef motor = new MotorSRX(null, 0, 0, false);
                this.testMiniMotors = new TestMiniMotors(driveHID, leds);
                leds.setColors(0, 128, 0);
                break;
            case Squidward: // Holiday Present Robot
                // Uses Talon SRX for drive train
                break;
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
        logf("Creating RobotContainer\n");
        if (Config.robotType != RobotType.Simulation) {
            configureButtonBindings();
        }
    }

    public void testLeds() {
        leds.setColors(0, 127, 0);
    }

    public double squareWithSign(double v) {
        return (v < 0) ? -(v * v) : (v * v);
    }

    // Command zy = new InstantCommand(new Runnable() {
    // public void run() {
    // Robot.yawProvider.zeroYaw();
    // testMotors.resetPos();
    // }
    // });

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
        //operatorHID.getAButton(); // Avoids a not used problem
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

    void deB() {
        // Creates a Debouncer in "both" mode.
        Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        // So if currently false the signal must go true for at least .1 seconds before
        // being read as a True signal.
        if (m_debouncer.calculate(input.get())) {
            logf("Input Changed:%b\n", input.get());
        }
    }
}
