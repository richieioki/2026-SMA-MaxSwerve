package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


//Currently negative on the rotation motor moves up and positive moves down
public class MoveIntake extends Command{
    
    private Intake m_intake;
    private int target;
    private int dir;

    public MoveIntake(Intake intake, int target) {
        m_intake = intake;
        this.target = target;
    }

    @Override
    public void initialize() {
        //dir = Math.signum((float)m_intake.getEncoderCount() - (float)target);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
