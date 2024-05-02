// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoPathways;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoCheckIntakeRetractedCommand;
import frc.robot.commands.AutoCommands.AutoDriveCommand;
import frc.robot.commands.AutoCommands.AutoIntakeCommand;
import frc.robot.commands.AutoCommands.AutoRunFeedCommand;
import frc.robot.commands.AutoCommands.AutoSpeakerAlignCommand;
import frc.robot.commands.AutoCommands.SetAutoModeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueL11N8N extends SequentialCommandGroup {
  /** Creates a new BlueL11N8N. */
  public BlueL11N8N(SwerveSubsystem m_swerve,IntakeSubsystem m_intake,ShooterSubsystem m_shooter,PhotonVisionSubsystem m_vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetAutoModeCommand(m_swerve,true),
      new AutoDriveCommand(m_swerve,0,1.3,40,false,1),
      new AutoSpeakerAlignCommand(m_swerve,m_shooter,m_vision,true),
      new AutoRunFeedCommand(m_shooter),
      new AutoSpeakerAlignCommand(m_swerve,m_shooter,m_vision,false),
      new AutoIntakeCommand(m_intake,true),
      new AutoDriveCommand(m_swerve,0,2.8,0,false,1),
      /*new TimerCommand(2),
      new AutoIntakeCommand(m_intake,false),*/
      new AutoCheckIntakeRetractedCommand(m_intake),
      new AutoDriveCommand(m_swerve,0,0,30,false,1),
      new AutoSpeakerAlignCommand(m_swerve,m_shooter,m_vision,true),
      new AutoRunFeedCommand(m_shooter),
      new AutoSpeakerAlignCommand(m_swerve,m_shooter, m_vision, false),
      new AutoIntakeCommand(m_intake,true),
      new AutoDriveCommand(m_swerve,-3,0,0,true,1.5),
      new AutoDriveCommand(m_swerve,-7,20,20,false,1),
      new AutoCheckIntakeRetractedCommand(m_intake),
      new AutoDriveCommand(m_swerve,2,-16,20,false,1),
      new AutoSpeakerAlignCommand(m_swerve,m_shooter,m_vision,true),
      new AutoRunFeedCommand(m_shooter),
      new AutoSpeakerAlignCommand(m_swerve, m_shooter, m_vision, false),
      new AutoIntakeCommand(m_intake,false),
      new SetAutoModeCommand(m_swerve,false)
    );
  }
}
