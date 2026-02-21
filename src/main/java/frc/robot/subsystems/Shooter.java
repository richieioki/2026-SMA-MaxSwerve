package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{

    private final Spark UpperShooter, LowerShooter, Feeder;

    public Shooter() {
        UpperShooter = new Spark(Constants.ShooterConstants.UpperShooterMotorPWM);
        LowerShooter = new Spark(Constants.ShooterConstants.LowerShooterMotorPWM);
        Feeder = new Spark(Constants.ShooterConstants.FeederMotorPWM);
    }

    public Command shootCommand() {
        return Commands.run(()-> runShooter());
    }

    public Command feederCommand() {
        return Commands.run(() -> runFeeder());
    }

    public Command stopShooting() {
        return Commands.run(() -> stop());
    }

    public void stop() {
        UpperShooter.set(0);
        LowerShooter.set(0);
        Feeder.set(0);
    }

    public void runShooter() {
        UpperShooter.set(Constants.ShooterConstants.ShooterSpeed);
        LowerShooter.set(Constants.ShooterConstants.ShooterSpeed);
    }
    
    public void runFeeder() {
        Feeder.set(Constants.ShooterConstants.FeederSpeed);
    }
}

