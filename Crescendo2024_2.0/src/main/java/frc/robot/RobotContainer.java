// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.PrepAmpCommand;
import frc.robot.commands.PrepClimbCommand;
import frc.robot.commands.PrepSpeakerCommand;
import frc.robot.commands.RetractCommand;
import frc.robot.commands.ShootAmpCommand;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.commands.ToggleLiftSolenoidCommand;
import frc.robot.commands.AutoCommands.AutoDriveCommand;
import frc.robot.commands.AutoPathways.TestPathway;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem m_swerveSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  ClimbSubsystem m_climbSubsystem;
  VisionSubsystem m_visionSubsystem;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick driverJoystick;
  Joystick operatorJoystick;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  DriveCommand m_driveCommand;
  DeployIntakeCommand m_deployIntakeCommand;
  PrepSpeakerCommand m_prepSpeakerCommand;
  PrepAmpCommand m_prepAmpCommand;
  PrepClimbCommand m_prepClimbCommand;
  ShootSpeakerCommand m_shootSpeakerCommand;
  ShootAmpCommand m_shootAmpCommand;
  LiftCommand m_liftCommand;
  RetractCommand m_retractCommand;
  ToggleLiftSolenoidCommand m_toggleLiftSolenoidCommand;
  //AUTONOMOUS
  TestPathway m_testPathway;
  //AUTONOMOUS COMMAND
  AutoDriveCommand m_autoDriveCommand;
  
  public RobotContainer() {
    ////SUBSYSTEMS////
    m_swerveSubsystem = new SwerveSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_intakeSubsystem = new IntakeSubsystem(m_shooterSubsystem);
    m_climbSubsystem = new ClimbSubsystem(m_intakeSubsystem, m_shooterSubsystem);
    m_visionSubsystem = new VisionSubsystem(m_swerveSubsystem, m_shooterSubsystem);
    ////CONTROLLERS////
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);
    ////COMMANDS////

    m_driveCommand = new DriveCommand(m_swerveSubsystem, driverJoystick);
    m_deployIntakeCommand = new DeployIntakeCommand(m_intakeSubsystem, m_shooterSubsystem);
    m_prepSpeakerCommand = new PrepSpeakerCommand(m_shooterSubsystem, m_visionSubsystem);
    m_prepAmpCommand = new PrepAmpCommand(m_shooterSubsystem);
    m_prepClimbCommand = new PrepClimbCommand(m_shooterSubsystem);
    m_shootSpeakerCommand = new ShootSpeakerCommand(m_shooterSubsystem);
    m_shootAmpCommand = new ShootAmpCommand(m_shooterSubsystem);
    m_liftCommand = new LiftCommand(m_climbSubsystem);
    m_retractCommand = new RetractCommand(m_climbSubsystem);
    m_toggleLiftSolenoidCommand = new ToggleLiftSolenoidCommand(m_climbSubsystem);
    //AUTO
    m_testPathway = new TestPathway(m_swerveSubsystem, m_intakeSubsystem, m_shooterSubsystem);
    //AUTO COMMANDS
    //m_autoDriveCommand = new AutoDriveCommand(m_swerveSubsystem, 0, 0, 0, false)
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    JoystickButton deployIntakeButton = new JoystickButton(operatorJoystick,4);
    JoystickButton prepSpeakerButton = new JoystickButton(operatorJoystick,6);
    JoystickButton prepAmpButton = new JoystickButton(operatorJoystick,5);
    JoystickButton prepClimbButton = new JoystickButton(operatorJoystick,9);
    JoystickButton shootSpeakerButton = new JoystickButton(operatorJoystick,3);
    JoystickButton shootAmpButton = new JoystickButton(operatorJoystick,1);
    JoystickButton liftButton = new JoystickButton(operatorJoystick,7);
    JoystickButton retractButton = new JoystickButton(operatorJoystick,8);
    JoystickButton toggleLiftSolenoidButton = new JoystickButton(operatorJoystick,2);
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    deployIntakeButton.toggleOnTrue(m_deployIntakeCommand);
    prepSpeakerButton.toggleOnTrue(m_prepSpeakerCommand);
    prepAmpButton.toggleOnTrue(m_prepAmpCommand);
    prepClimbButton.toggleOnTrue(m_prepClimbCommand);
    shootSpeakerButton.whileTrue(m_shootSpeakerCommand);
    shootAmpButton.whileTrue(m_shootAmpCommand);
    liftButton.whileTrue(m_liftCommand);
    retractButton.whileTrue(m_retractCommand);
    toggleLiftSolenoidButton.onTrue(m_toggleLiftSolenoidCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return m_testPathway;
  }
  public Command getTeleopCommand(){
    return m_driveCommand;
  }
}
