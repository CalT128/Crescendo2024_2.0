// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetRobotCommand extends InstantCommand {
  SwerveSubsystem m_swerve;
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  PhotonVisionSubsystem m_vision;
  
  public ResetRobotCommand(SwerveSubsystem swerve, IntakeSubsystem intake, ShooterSubsystem shooter,PhotonVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_intake = intake;
    m_shooter = shooter;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetOnce();
    m_swerve.setAutoMode(false);
    m_swerve.setLockedOn(false);
    m_intake.setAutoIntakeMode(false);
    m_vision.setAutoAlign(false);
  }
}
