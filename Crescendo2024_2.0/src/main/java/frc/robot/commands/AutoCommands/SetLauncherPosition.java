// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLauncherPosition extends InstantCommand {
  ShooterSubsystem m_shooter;
  ShooterPosition pos;
  public SetLauncherPosition(ShooterSubsystem shooter,ShooterPosition position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    position = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterPosition(pos);
  }
}
