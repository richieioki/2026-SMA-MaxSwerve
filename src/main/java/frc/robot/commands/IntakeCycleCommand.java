package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.INTAKESTATE;

public class IntakeCycleCommand extends Command {
    private final Intake m_intake;
    private PIDController rotatorPID;

    // States: 0=Up, 1=Down, 2=Halfway
    private int counter;

    public IntakeCycleCommand(Intake intake) {
        m_intake = intake;
        // rotatorPID = intake.getPidController();
        counter = 0;
    }

    @Override
    public void initialize() {

        // if (m_intake.m_state == INTAKESTATE.UP) {
        //     m_intake.setTargetPosition(IntakeConstants.EncoderDownPosition);
        // }

        // if (m_intake.m_state == INTAKESTATE.DOWN) {
        //     m_intake.setTargetPosition(IntakeConstants.EncoderHalfwayPosition);
        // }

        // if (m_intake.m_state == INTAKESTATE.MID) {
        //     m_intake.setTargetPosition(IntakeConstants.EncoderUpPosition);
        // }
    }

    @Override
    public void execute() {
        if (m_intake.m_state == INTAKESTATE.UP) {
            if (counter <= 3) {
                m_intake.setRotatorSpeed(0.5);
                counter++;
            } else {
                m_intake.setRotatorSpeed(-0.07);
            }
        } else if(m_intake.m_state == INTAKESTATE.DOWN) {
            m_intake.setRotatorSpeed(-0.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setRotatorSpeed(0);

        if (m_intake.m_state == INTAKESTATE.UP) {
            m_intake.m_state = INTAKESTATE.DOWN;
        }

        else if (m_intake.m_state == INTAKESTATE.DOWN) {
            m_intake.m_state = INTAKESTATE.MID;
        }

        else if (m_intake.m_state == INTAKESTATE.MID) {
            m_intake.m_state = INTAKESTATE.UP;
        }
    }

    @Override
    public boolean isFinished() {
        return m_intake.onTarget();
    }
}
