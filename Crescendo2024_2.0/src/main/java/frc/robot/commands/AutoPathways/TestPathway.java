// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoPathways;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoDriveCommand;
import frc.robot.commands.AutoCommands.AutoIntakeCommand;
import frc.robot.commands.AutoCommands.AutoRunFeedCommand;
import frc.robot.commands.AutoCommands.AutoSpeakerAlignCommand;

import frc.robot.commands.AutoCommands.SetAutoModeCommand;
import frc.robot.commands.AutoCommands.TimerCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPathway extends SequentialCommandGroup {
  /** Creates a new TestPathway. */
  SwerveSubsystem m_swerve;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  PhotonVisionSubsystem m_vision;
  public TestPathway(SwerveSubsystem swerve, IntakeSubsystem intake, ShooterSubsystem shooter,PhotonVisionSubsystem vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_swerve = swerve;
    m_intake = intake;
    m_shooter = shooter;
    addCommands(
      new SetAutoModeCommand(m_swerve,true),
      new AutoDriveCommand(m_swerve,-1,3,0,true,1.5),
      new AutoDriveCommand(m_swerve,1,3,0,false,1),
      new SetAutoModeCommand(m_swerve,false)
      //new AutoDriveCommand(m_swerve,-3,3,0,false)
      //new AutoSpeakerAlignCommand(shooter, m_vision,true),
      //new AutoRunFeedCommand(shooter)
      /*new AutoDriveCommand(m_swerve,0,2.4,0,false),
      new AutoDriveCommand(m_swerve,0,0,30,false),
      new AutoDriveCommand(m_swerve,3.5,-2,0,false),
      new AutoDriveCommand(m_swerve,1,2,0,false),
      new AutoDriveCommand(m_swerve,4,-2,330,false)*/
    );
  }
}

