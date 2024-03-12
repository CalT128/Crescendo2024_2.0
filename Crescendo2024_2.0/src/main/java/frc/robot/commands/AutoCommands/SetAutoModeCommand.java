// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAutoModeCommand extends InstantCommand {
  SwerveSubsystem m_swerve;
  boolean autoMode;
  public SetAutoModeCommand(SwerveSubsystem swerve, boolean autoMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    this.autoMode = autoMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.setAutoMode(autoMode);
  }
}
