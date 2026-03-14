// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveBackwardsCommand;
import frc.robot.commands.PointToPointPID;

import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_Intake = new Intake();
  private final Climber m_climber = new Climber();  

  final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    m_chooser.addOption("Simple Back and Shoot", 
      new SequentialCommandGroup(
        new DriveBackwardsCommand(m_robotDrive),
        new ShooterCommand(m_shooter).withTimeout(3.0)
    ));
    
    m_chooser.addOption("Test P2P",
      new PointToPointPID(m_robotDrive, new Pose2d(-1.5, 0, Rotation2d.fromDegrees(0))));


    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //*************SMA Commands **************************************/
    
    // Right Trigger (Hold) -> Shoot
    m_driverController.rightTrigger(0.5).whileTrue(new ShooterCommand(m_shooter));

    // Right Bumper (Hold) -> Spin Intake
    m_driverController.rightBumper()
        .whileTrue(m_Intake.runIntake())
        .onFalse(Commands.runOnce(() -> m_Intake.stopIntake()));

    // Left Bumper (Press) -> Cycle Intake Position (Up -> Down -> Halfway)
    m_driverController.leftBumper().onTrue(new frc.robot.commands.IntakeCycleCommand(m_Intake));

    // Y Button (Hold) -> Extend Climber (Placeholder)
    m_driverController.y().onTrue(Commands.runOnce(()-> m_Intake.reverseIntake()))
      .onFalse(Commands.runOnce(() -> m_Intake.stopIntake()));
        

    // A Button (Hold) -> Retract Climber (Placeholder)
    m_driverController.a().onTrue(Commands.runOnce(()->m_shooter.reverseShooter()))
      .onFalse(Commands.runOnce(()->m_shooter.stop()));
        

    m_driverController.start().onTrue(Commands.run(() -> m_robotDrive.zeroHeading()));
  }

  /**
   * Registers PathPlanner NamedCommands so they can be triggered from the GUI.
   */

  private void registerNamedCommands() {
    // Register the shooting command. We use withTimeout to ensure it doesn't run forever if it's not interrupted,
    // though PathPlanner handles command lifecycle internally as well.
    NamedCommands.registerCommand("Shoot", new ShooterCommand(m_shooter).withTimeout(2.0));

    // Register our new automated climbing sequence! 
    NamedCommands.registerCommand("Climb", new ClimbCommand(m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return a timed sequence: Drive backwards for 2 seconds (enforced), then shoot for up to 3 seconds
    // return new SequentialCommandGroup(
    //     new DriveBackwardsCommand(m_robotDrive).withTimeout(2.5),
    //     new ShooterCommand(m_shooter).withTimeout(3.0)
    // );
    return m_chooser.getSelected();

  }

  public void testPeriodic() {

    if(m_driverController.povUp().getAsBoolean())
      m_Intake.IntakeTest(-0.3 );
    else  if(m_driverController.povDown().getAsBoolean())
      m_Intake.IntakeTest(0.3 );
    else 
      m_Intake.IntakeTest(0 );
  }
}
