// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    // 1. Determine Alliance side for correct starting pose location
    // We default to blue if it cannot be read
    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    
    // 2. Define starting pose. Adjust X/Y to actual field starting locations if known!
    // Example: Red starts at X=14.5, Blue starts at X=2.0
    Pose2d startingPose;
    if (isRedAlliance) {
        // Red Alliance: Facing negative X (180 degrees)
        startingPose = new Pose2d(14.5, 5.5, Rotation2d.fromDegrees(180));
    } else {
        // Blue Alliance: Facing positive X (0 degrees)
        startingPose = new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0));
    }

    // 3. Reset the robot's odometry to the chosen starting pose
    m_robotDrive.resetOdometry(startingPose);

    // 4. We want to drive backwards 2 meters relative to the new starting location
    // We add/subtract based on field coordinates
    Pose2d targetPose;
    if (isRedAlliance) {
        // Backwards for red is POSITIVE X
        targetPose = new Pose2d(
            startingPose.getX() + 2.0, 
            startingPose.getY(), 
            startingPose.getRotation()
        );
    } else {
        // Backwards for blue is NEGATIVE X
        targetPose = new Pose2d(
            startingPose.getX() - 2.0, 
            startingPose.getY(), 
            startingPose.getRotation()
        );
    }

    // 5. Create a path finding command to that target
    Command driveBackwardCmd = AutoBuilder.pathfindToPose(
        targetPose, 
        new com.pathplanner.lib.path.PathConstraints(
            3.0, 3.0, 
            edu.wpi.first.math.util.Units.degreesToRadians(360), 
            edu.wpi.first.math.util.Units.degreesToRadians(360)
        ),
        0.0 // Goal end velocity
    );

    // 6. Return the sequential command group (Drive then Shoot)
    return new SequentialCommandGroup(
        driveBackwardCmd,
        new ShooterCommand(m_shooter)
    );
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
