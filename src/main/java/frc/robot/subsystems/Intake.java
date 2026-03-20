package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public enum INTAKESTATE {
        UP, DOWN, MID, GOINGDOWN
    };

    public INTAKESTATE m_state;
    private int counter = 0;

    private final Spark OuterInake;
    private final SparkMax InnerIntake, Rotator2;
    public Encoder intakEncoder = new Encoder(Constants.IntakeConstants.EncoderChannelA,
            Constants.IntakeConstants.EncoderChannelB);
    public CommandXboxController controller;

    private PIDController rotatorPID = new PIDController(0.0028, 0, 0);
    // private ArmFeedforward armFeedforward = new ArmFeedforward(0, -0.08, 0);
    public int targetPosition = Constants.IntakeConstants.EncoderUpPosition;

    public Intake(CommandXboxController controller) {
        this.controller = controller;
        m_state = INTAKESTATE.UP;

        OuterInake = new Spark(Constants.IntakeConstants.OutsideIntakePWM);
        // InnerIntake = new Spark(Constants.IntakeConstants.InsideIntakePWM);
        InnerIntake = new SparkMax(IntakeConstants.InsideIntakeCAN, MotorType.kBrushless);
        Rotator2 = new SparkMax(IntakeConstants.RotatorID2, MotorType.kBrushless);

        SparkMaxConfig SparkMaxConfigLeader = new SparkMaxConfig();
        SparkMaxConfigLeader.idleMode(IdleMode.kBrake);
        Rotator2.configure(SparkMaxConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // SparkMaxConfig SparkMaxConfigFollow = new SparkMaxConfig();
        // SparkMaxConfigFollow.apply(SparkMaxConfigLeader);
        // SparkMaxConfigFollow.follow(Rotator1);
        // Rotator2.configure(SparkMaxConfigFollow, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);

        // rotatorPID.setTolerance(Constants.IntakeConstants.kPositionTolerance);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake Encoder", intakEncoder.get());
        SmartDashboard.putNumber("Intake Target", targetPosition);

        if (DriverStation.isTest()) {
            if (controller.povDown().getAsBoolean()) {
                setRotatorSpeed(0.1d); // positive is down
            } else if (controller.povUp().getAsBoolean()) {
                setRotatorSpeed(-0.1d); // negative is up
            } else {
                rotateStop();
            }
        } else {

            setRotatorSpeed(-MathUtil.clamp(rotatorPID.calculate(intakEncoder.get(), targetPosition), -0.75, 0.75));

            if (m_state == INTAKESTATE.GOINGDOWN) {
                if (intakEncoder.get() <= targetPosition + Constants.IntakeConstants.kPositionTolerance) {
                    m_state = INTAKESTATE.DOWN;
                }
            } 

            // if (m_state == INTAKESTATE.GOINGDOWN) {
            // setRotatorSpeed(Constants.IntakeConstants.armSpeed);

            // if (intakEncoder.get() <= targetPosition) {
            // m_state = INTAKESTATE.DOWN;
            // rotateStop();
            // }
            // } else if (m_state == INTAKESTATE.MID) {
            // if (intakEncoder.get() < targetPosition)
            // setRotatorSpeed(-Constants.IntakeConstants.armSpeed);
            // else
            // rotateStop();
            // } else if (m_state == INTAKESTATE.UP) {
            // if (intakEncoder.get() < targetPosition) {
            // setRotatorSpeed(-Constants.IntakeConstants.midtotopspeed);
            // } else {
            // rotateStop();
            // }
            // } else {
            // rotateStop();
            // }
        }

    }

    // if (onTarget() && m_state == INTAKESTATE.GOINGDOWN) {
    // m_state = INTAKESTATE.DOWN;
    // }

    // public void setTargetPosition(int target) {
    // this.targetPosition = target;
    // }

    // public boolean isAtTarget() {
    // return rotatorPID.atSetpoint();
    // }

    public Command runIntake() {
        return Commands.run(() -> spinIntake(), this);
    }

    public void spinIntake() {
        // Only spin the rollers if the intake is deployed (Target is Down)
        if (targetPosition == Constants.IntakeConstants.EncoderDownPosition) {
            OuterInake.set(-Constants.IntakeConstants.intakeSpeedouter);
            InnerIntake.set(-Constants.IntakeConstants.intakeSpeed);
        } else {
            stopIntake();
        }
    }

    public void reverseIntake() {
        // Only spin the rollers if the intake is deployed (Target is Down)
        if (targetPosition == Constants.IntakeConstants.EncoderDownPosition) {
            OuterInake.set(Constants.IntakeConstants.intakeSpeedouter);
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
        Rotator2.set(0);
    }

    public int getEncoderCount() {
        return intakEncoder.get();
    }

    public void setRotatorSpeed(double speed) {
        Rotator2.set(speed);
    }

    public void ToggleIntakeState() {
        if (m_state == INTAKESTATE.UP) {
            m_state = INTAKESTATE.GOINGDOWN;
            targetPosition = IntakeConstants.EncoderDownPosition;
            counter = 0;
        } else if (m_state == INTAKESTATE.DOWN) {
            m_state = INTAKESTATE.MID;
            targetPosition = IntakeConstants.EncoderHalfwayPosition;
        } else if (m_state == INTAKESTATE.MID) {
            m_state = INTAKESTATE.UP;
            targetPosition = IntakeConstants.EncoderUpPosition;
        }
    }

    public boolean onTarget() {
        // TODO Auto-generated method stub
        return false;
        // intakEncoder.get() > (targetPosition - 10) &&
        // intakEncoder.get() < (targetPosition + 10);
    }
}
