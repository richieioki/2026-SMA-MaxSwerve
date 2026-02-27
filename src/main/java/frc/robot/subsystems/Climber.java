package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // Replace 10 with your actual CAN ID later
    private final SparkMax m_climberMotor; 

    public Climber() {
        m_climberMotor = new SparkMax(10, MotorType.kBrushless);
        // Add configurations like inversion, current limits, etc., here later
    }

    /** 
     * Extends the climber at a set speed.
     * Climber in a box usually extends out to hook the chain/bar.
     */
    public void extend() {
        m_climberMotor.set(0.5); 
    }

    /** 
     * Retracts the climber to pull the robot up.
     */
    public void retract() {
        m_climberMotor.set(-1.0); // Full power retract
    }

    /**
     * Stops the climber motor.
     */
    public void stop() {
        m_climberMotor.set(0.0);
    }
}
