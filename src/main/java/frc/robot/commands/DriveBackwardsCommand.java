package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveBackwardsCommand extends Command {
    private final DriveSubsystem m_drive;
    private double startTime;
    private final double duration = 2.0; // Seconds to drive backward

    /**
     * Creates a new DriveBackwardsCommand.
     *
     * @param drive The drive subsystem used by this command.
     */
    public DriveBackwardsCommand(DriveSubsystem drive) {
        m_drive = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Drive straight backward at 20% speed (assuming positive X is forward)
        // Adjust the -0.2 value depending on how fast you want the robot to move. 
        // 0.0 is Y (Strafe) and 0.0 is Rot (Turn)
        m_drive.drive(-0.2, 0.0, 0.0, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        m_drive.drive(0.0, 0.0, 0.0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= duration;
    }
}
