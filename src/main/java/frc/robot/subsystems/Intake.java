package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final Spark OuterInake, InnerIntake, Rotator;
    private Encoder intakEncoder = new Encoder(Constants.IntakeConstants.EncoderChannelA, 
        Constants.IntakeConstants.EncoderChannelB);

    private final PIDController rotatorPID = new PIDController(0.01, 0, 0); 
    private int targetPosition = Constants.IntakeConstants.EncoderUpPosition;

    public Intake() {
        OuterInake = new Spark(Constants.IntakeConstants.OutsideIntakePWM);
        InnerIntake = new Spark(Constants.IntakeConstants.InsideIntakePWM);
        Rotator = new Spark(Constants.IntakeConstants.RotatorPWM);

        rotatorPID.setTolerance(Constants.IntakeConstants.kPositionTolerance);
    }

    @Override
    public void periodic() {
        super.periodic();

        double output = rotatorPID.calculate(intakEncoder.get(), targetPosition);
        output = edu.wpi.first.math.MathUtil.clamp(output, -0.6, 0.6);
        Rotator.set(output);

        SmartDashboard.putNumber("Intake Encoder", intakEncoder.get());
        SmartDashboard.putNumber("Intake Target", targetPosition);
    }

    public void setTargetPosition(int target) {
        this.targetPosition = target;
    }

    public boolean isAtTarget() {
        return rotatorPID.atSetpoint();
    }

    public Command runIntake() {
        return Commands.run(() -> spinIntake(), this);
    }

    public void spinIntake() {
        // Only spin the rollers if the intake is deployed (Target is Down)
        if (targetPosition == Constants.IntakeConstants.EncoderDownPosition) {
            OuterInake.set(-Constants.IntakeConstants.intakeSpeedouter);
            InnerIntake.set(Constants.IntakeConstants.intakeSpeed);
        } else {
            stopIntake();
        }
    }

    public void stopIntake() {
        OuterInake.set(0);
        InnerIntake.set(0);
    }

    public void rotateStop() {
        Rotator.set(0);
    }

    public int getEncoderCount() {
        return intakEncoder.get();
    }
}

