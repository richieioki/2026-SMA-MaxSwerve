package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
    
    private double starttime;
    private double delaytime = 0.5f;
    private Shooter m_shooter;

    public ShooterCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        starttime = Timer.getFPGATimestamp();
        m_shooter.runShooter();
    }

    @Override
    public void execute() {
        super.execute();

        SmartDashboard.putBoolean("Feeder on", Timer.getFPGATimestamp() > (starttime + delaytime));

        if(Timer.getFPGATimestamp() > (starttime + delaytime))
            m_shooter.runFeeder();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

}
