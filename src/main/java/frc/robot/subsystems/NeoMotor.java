package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.ClosedLoopSlot.pid ; 


import static frc.robot.utilities.Util.logf;

public class NeoMotor extends SubsystemBase {
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private XboxController driveHID; 
   // private SparkPIDController m_pidController;

    public NeoMotor(XboxController driveHID) {
        int motorID = 20; // Replace with the actual CAN ID of your NEO
        m_motor = new SparkMax(motorID, MotorType.kBrushless);
        this.driveHID = driveHID;
        // Factory reset, just in case
        //m_motor.restoreFactoryDefaults();

        // Get the encoder object
        m_encoder = m_motor.getEncoder();

        // Initialize PID controller
        //m_pidController = m_motor.getPIDController();

        // Optional: Set PID coefficients (tune these for your application)
        // double kP = 0.1;
        // double kI = 0;
        // double kD = 0;
        // double kIz = 0;
        // double kFF = 0;
        // double kMaxOutput = 1;
        // double kMinOutput = -1;

        // m_pidController.setP(kP);
        // m_pidController.setI(kI);
        // m_pidController.setD(kD);
        // m_pidController.setIZone(kIz);
        // m_pidController.setFF(kFF);
        // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // Optional: Set the idle mode (kCoast or kBrake)
        //m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
  public void periodic() {
    double value = driveHID.getLeftTriggerAxis();
    logf("NEO Value:%.2f\n", value);
    m_motor.set(value);

  }
}
