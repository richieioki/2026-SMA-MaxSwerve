package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{

    private final Spark UpperShooter, LowerShooter, Feeder;

    public Shooter() {
        UpperShooter = new Spark(Constants.ShooterConstants.UpperShooterMotorPWM);
        LowerShooter = new Spark(Constants.ShooterConstants.LowerShooterMotorPWM);
        Feeder = new Spark(Constants.ShooterConstants.FeederMotorPWM);

        Feeder.set(0);
    }

    public void stop() {
        UpperShooter.set(0);
        LowerShooter.set(0);
        Feeder.set(0);
    }

    public void runShooter() {
        UpperShooter.set(-Constants.ShooterConstants.ShooterSpeedupper);
        LowerShooter.set(Constants.ShooterConstants.ShooterSpeed);
    }
    
    public void runFeeder() {
        Feeder.set(Constants.ShooterConstants.FeederSpeed);
    }
}

