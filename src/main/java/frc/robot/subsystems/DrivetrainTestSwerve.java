package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainTestSwerve extends SubsystemBase {
    private int[] stearMotor = { 1, 4, 7, 10 };
    private int[] canCoder = { 2, 5, 8, 11 };
    private int[] driveMotor = { 3, 6, 9, 12 };
    private final int numberOfMotors = 4;
    private MotorKraken[] motorFlex = new MotorKraken[numberOfMotors];
    private MotorSparkMax[] motorSpark = new MotorSparkMax[ numberOfMotors];
    private CanCoderSubsystem[] canCoders = new CanCoderSubsystem[numberOfMotors];
    private XboxController driveController;
    private boolean lastStart = false;
    private TestType testType = TestType.FR;

    public DrivetrainTestSwerve(XboxController driveController) {
        this.driveController = driveController;
        for (int i = 0; i < numberOfMotors; i++) {
            motorFlex[i] = new MotorKraken("Drive", driveMotor[i], -1, true);
            motorSpark[i] = new MotorSparkMax("Stear" + i, stearMotor[i], -1, false, false);
            motorSpark[i].setLogging(true);
            canCoders[i] = new CanCoderSubsystem("CC"+i, canCoder[i], false);
        }
    }

    enum TestType {
        FR, FL, RR, RL, ALL;

        public TestType next() {
            TestType[] values = TestType.values();
            int nextOrdinal = (this.ordinal() + 1) % values.length;
            return values[nextOrdinal];
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Make sure that you declare this subsystem in RobotContainer.java
        boolean start = driveController.getStartButton();
        if (start && !lastStart) {
            setSpeedAll(0.0);
            testType = testType.next(); // Get the next mode
            logf("New Test Type:%s\n", testType);
        }
        SmartDashboard.putString("Type", testType.toString());
        double leftJoy = driveController.getLeftY();
        double rightJoy = driveController.getRightY();
        if (testType == TestType.ALL) {
            for (int i = 0; i < numberOfMotors; i++) {
                motorFlex[i].setSpeed(leftJoy);
                motorSpark[i].setSpeed(rightJoy);
            }
        } else {
            motorFlex[testType.ordinal()].setSpeed(leftJoy);
            motorSpark[testType.ordinal()].setSpeed(rightJoy);
        }
        // Show Cancoder angle on SmartDashboard
        for (int i = 0; i < numberOfMotors; i++) {
            SmartDashboard.putNumber("CC" + i, canCoders[i].getAngle());
        }
    }

    private void setSpeedAll(double speed) {
        for (int i = 0; i < numberOfMotors; i++) {
            motorFlex[i].setSpeed(speed);
            motorSpark[i].setSpeed(speed);
        }
    }
}
