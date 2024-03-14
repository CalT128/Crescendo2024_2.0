// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoPathways;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoDriveCommand;
import frc.robot.commands.AutoCommands.AutoIntakeCommand;
import frc.robot.commands.AutoCommands.AutoRunFeedCommand;
import frc.robot.commands.AutoCommands.AutoSpeakerAlignCommand;
import frc.robot.commands.AutoCommands.SetAutoIntakeModeCommand;
import frc.robot.commands.AutoCommands.SetAutoModeCommand;
import frc.robot.commands.AutoCommands.TimerCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedR3N extends SequentialCommandGroup {
  /** Creates a new RedR3N. */
  public RedR3N(SwerveSubsystem m_swerve,IntakeSubsystem m_intake,ShooterSubsystem m_shooter,PhotonVisionSubsystem m_vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RedR(m_swerve, m_intake, m_shooter, m_vision),
      new SetAutoModeCommand(m_swerve,true),
      new SetAutoIntakeModeCommand(m_intake, m_shooter,true),
      new AutoIntakeCommand(m_intake,true),
      new AutoDriveCommand(m_swerve,0,3,0,false),
      new TimerCommand(2),
      new SetAutoIntakeModeCommand(m_intake,m_shooter,false),
      new AutoDriveCommand(m_swerve,0,0,330,false),
      new AutoSpeakerAlignCommand(m_shooter,m_vision,true),
      new AutoRunFeedCommand(m_shooter),
      new AutoSpeakerAlignCommand(m_shooter,m_vision,false),
      new AutoIntakeCommand (m_intake,false),
      new SetAutoModeCommand(m_swerve,false)
      
    );
  }
}
