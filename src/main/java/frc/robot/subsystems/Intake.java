package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final Spark OuterInake, InnerIntake, Rotator;
    private final CommandXboxController controller;

    public Intake(CommandXboxController controller) {
        OuterInake = new Spark(Constants.IntakeConstants.OutsideIntakePWM);
        InnerIntake = new Spark(Constants.IntakeConstants.InsideIntakePWM);
        Rotator = new Spark(Constants.IntakeConstants.RotatorPWM);
        this.controller = controller;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        Rotator.set(0.5 * controller.getRawAxis(5));
    }

    public Command runIntake() {
        return Commands.run(() -> spinIntake());
    }

    public Command RotateUp() {
        return Commands.run(()-> rotateUp());
    }

    public Command RotateDown() {
        return Commands.run(()-> rotateDown());
    }

    public void spinIntake() {
        OuterInake.set(Constants.IntakeConstants.intakeSpeed);
        InnerIntake.set(Constants.IntakeConstants.intakeSpeed);
    }

    public void stopIntake() {
        OuterInake.set(0);
        InnerIntake.set(0);
    }

    public void rotateUp() {
        Rotator.set(0.1);
    }

    public void rotateDown() {
        Rotator.set(-0.1);
    }

    public void rotateStop() {
        Rotator.set(0);
    }
}
