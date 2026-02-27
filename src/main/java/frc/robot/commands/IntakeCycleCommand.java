package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCycleCommand extends Command {
    private final Intake m_intake;
    
    // States: 0=Up, 1=Down, 2=Halfway
    private static int currentStateIndex = 0; 
    
    public IntakeCycleCommand(Intake intake) {
        m_intake = intake;
        // We do NOT addRequirements(m_intake) because this command only INSTANTLY sets the target variable, 
        // it doesn't hold the motor hostage (the internal PID in Intake.periodic() handles that).
    }

    @Override
    public void initialize() {
        // Cycle to the next state (Up -> Down -> Halfway -> Up)
        currentStateIndex = (currentStateIndex + 1) % 3;

        switch (currentStateIndex) {
            case 0: // Up
                m_intake.setTargetPosition(IntakeConstants.EncoderUpPosition);
                break;
            case 1: // Down
                m_intake.setTargetPosition(IntakeConstants.EncoderDownPosition);
                break;
            case 2: // Halfway
                m_intake.setTargetPosition(IntakeConstants.EncoderHalfwayPosition);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true; // Command finishes instantly; PID takes over in Intake.periodic()
    }
}
