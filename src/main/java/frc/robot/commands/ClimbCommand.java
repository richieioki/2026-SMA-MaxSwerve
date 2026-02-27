package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends Command {
    private final Climber m_climber;

    public ClimbCommand(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void execute() {
        // PathPlanner auto sequence will trigger this to retract the climber
        m_climber.retract();
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }

    @Override
    public boolean isFinished() {
        // Placeholder: Usually you'd check a limit switch or encoder position here
        // to stop the motor when fully retracted.
        return false; 
    }
}
