package frc.robot;

// import static frc.robot.Robot.yaw;
import static frc.robot.utilities.Util.logf;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config.RobotType;
import frc.robot.subsystems.DrivetrainJaguar;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.DrivetrainSpark;
import frc.robot.subsystems.DrivetrainTestSwerve;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.MotorFlex;
import frc.robot.subsystems.MotorKraken;
import frc.robot.subsystems.MotorSRX;
import frc.robot.subsystems.MotorSparkMax;
import frc.robot.subsystems.TestTriggers;

/**
 * This class is where the bulk of the robot should be declared. Since be
 * declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead,
 * the structure ofthe
 * robot (including subsystems, commands, and button mappings) should be
 * declared here.
 */
public class RobotContainer {
  public static final CommandXboxController driveController = new CommandXboxController(2);
  private static final XboxController driveHID = driveController.getHID();

  public static LedSubsystem leds = new LedSubsystem();
  public DrivetrainSRX drivetrainSRX;

  private TestTriggers triggers = new TestTriggers();
  private CANcoder canCoder;

  public double getSpeedFromTriggers() {
    double leftValue = driveController.getLeftTriggerAxis();
    double rightValue = driveController.getRightTriggerAxis();
    if (leftValue > 0.05) {
      return leftValue;
    }
    if (rightValue > 0.05) {
      return -rightValue;
    }
    return 0.0;
  }


  enum Modes {
    POSITION, VELOCITY, MOTIONMAGIC, SPEED;

    public Modes next() {
      Modes[] values = Modes.values();
      int nextOrdinal = (this.ordinal() + 1) % values.length;
      return values[nextOrdinal];
    }
  }

  enum MiniSRXMotors {TestFlex, TestMax, TestKrak, TestSRX;

    public MiniSRXMotors next() {
      MiniSRXMotors[] values = MiniSRXMotors.values();
      int nextOrdinal = (     this.ordinal() + 1) % values.length;
      return values[nextOrdinal];
    }
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
        boolean testSmartMaxBlonde = true;
        MotorSparkMax motor = new MotorSparkMax("TestMax", 20, -1, false, false);
        if (testSmartMaxBlonde) {
          motor.setLogging(true);
          motor.setTestMode(true);
        } else {
          Command blondeMove = Commands.run(() -> motor.setSpeed(getSpeedFromTriggers()), motor);
          blondeMove.ignoringDisable(true).schedule();
        }
        break;
      case DarrylMini:
        new DrivetrainSRX(driveHID);
        MotorSRX dmotor = new MotorSRX("DarrylSRX", 10, -1, true);
        Command darrylMoveBack = Commands.run(() -> dmotor.setSpeed(getSpeedFromTriggers()), dmotor);
        darrylMoveBack.ignoringDisable(true).schedule();
        break;
      case MiniMini:
        MotorSRX mmmotor = new MotorSRX("MiniSRX", 10, -1, true);
        Command miniMove = Commands.run(() -> mmmotor.setSpeed(driveController.getLeftTriggerAxis()), mmmotor);
        driveController.start().onTrue(miniMove);
        new ScheduleCommand(miniMove);
        break;
      case MiniKeith: // Test mini
        // Use Talon SRX for drive train
        drivetrainSRX = new DrivetrainSRX(driveHID);
        // Setup to test Flex Motor
        boolean testFlex = false;
        if (testFlex) {
          MotorFlex mmotor = new MotorFlex("TestFlex", 10, -1, false);
          mmotor.setLeds(leds);
          mmotor.setLogging(true);
          mmotor.setTestMode(true);
        }
        boolean testSmartMax = false;
        if (testSmartMax) {
          MotorSparkMax mmotor = new MotorSparkMax("TestMax", 11, -1, false, false);
          mmotor.setLogging(true);
          mmotor.setTestMode(true);
        }
        boolean testKraken = true;
        if (testKraken) {
          MotorKraken motorK = new MotorKraken("TestKrak", 16, -1, true);
          motorK.setLogging(true);
          motorK.setTestMode(true);
        }
        boolean testSRX = false;
        boolean testRedMotor = true;
        if (testSRX) {
          MotorSRX motorSRX = new MotorSRX("SRX", 14, 0, true);
          if(testRedMotor) {
          motorSRX.setupForTestCasesRedMotor(leds);
          } else {
            motorSRX.setupForTestCasesGrayMotor(leds);
          }
          motorSRX.setLogging(true);
        }
        // Command miniSRXMove = Commands.run(() ->
        // motor.setSpeed(getSpeedFromTriggers()), motor);
        canCoder = new CANcoder(20);
        Command miniCancoder = Commands.run(
            () -> SmartDashboard.putNumber("CanCo", canCoder.getPosition().getValueAsDouble()));
        miniCancoder.ignoringDisable(true).schedule();
        // driveController.back().onTrue(miniCancoder);
        break;
      case Squidward:
        drivetrainSRX = new DrivetrainSRX(driveHID);
        // Uses Talon SRX for drive train())
        break;
      case Kevin: // Ginger Bread Robot
        // Uses Talon SRX for drive train
        drivetrainSRX = new DrivetrainSRX(driveHID);
        break;
      case Wooly: // Big ball shooter
        // Uses Jaguars for drive train and shooter
        // Uses PCM to control shooter tilt and shooter activate
        Robot.config.driveTrainJaguar = true;
        Robot.config.enableCompressor = true;
        Robot.config.pneumaticType = PneumaticsModuleType.CTREPCM;
        new DrivetrainJaguar(driveHID);
        break;
      case Mando: // Train engine
        // Use SparkMax motors for drive train
        new DrivetrainSpark(driveHID);
        break;
      case Sibling2025:
        new DrivetrainTestSwerve(driveHID);
        break;
    }
    logf("Finished Creating RobotContainer\n");
    if (Config.robotType != RobotType.Simulation) {
      configureButtonBindings();
    }
    SmartDashboard.putData("UpdatePID", hit);
  }

  Command h = Commands.run(() -> logf("Hit\f"));

  Command hit = new InstantCommand(
      new Runnable() {
        public void run() {
          logf("Hit Button\n");
        }
      });

  // private Command controlFlex(MotorDef motor) {
  // return Commands.run(() ->
  // motor.setSpeed(driveController.getLeftTriggerAxis()));
  // }

  public double squareWithSign(double v) {
    return (v < 0) ? -(v * v) : (v * v);
  }

  // The following code is an attempt to learn how to program commands

  public class hitButton extends Command {
    @Override
    public void execute() {
      // Your code to run when the button is pressed, such as moving a motor
      System.out.println("My command is running!");
    }
  }

  Command zy = new InstantCommand(
      new Runnable() {
        public void run() {
          Robot.yawProvider.zeroYaw();
          // testMotors.resetPos();
        }
      });

  Command testCmd = new InstantCommand(
      new Runnable() {
        public void run() {
          logf("Hit x on Game Pad\n");
        }
      });

  Command testTrigger = new InstantCommand(
      new Runnable() {
        public void run() {
          logf("Switch Hit\n");
        }
      });

  Trigger tr = new Trigger(triggers::getSwitch);

  private void configureButtonBindings() {
    // operatorHID.getAButton(); // Avoids a not used problem
    driveController.x().onTrue(testCmd);
    driveController
        .back()
        .whileTrue(
            new InstantCommand(
                new Runnable() {
                  public void run() {
                    Robot.yawProvider.zeroYaw();
                    logf("Hit back on Game Pad\n");
                  }
                }));
    // driveController.start().onTrue(zy);
    tr.onTrue(testTrigger);
  }

  // Initializes a DigitalInput
  DigitalInput input = new DigitalInput( Robot.config.DIOTestTrigger);
  // Creates a Debouncer in "both" mode.
  Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  void deB() {
    // If false the signal must go true for at least .1 seconds before read
    if (m_debouncer.calculate(input.get())) {
      logf("Input Changed:%b\n", input.get());
    }
  }
}
