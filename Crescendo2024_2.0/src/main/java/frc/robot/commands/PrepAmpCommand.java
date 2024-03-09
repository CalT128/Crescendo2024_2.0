// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepAmpCommand extends Command {
  /** Creates a new PrepAmpCommand. */
  ShooterSubsystem m_shooter;
  boolean conflict;
  boolean isFinished;
  public PrepAmpCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    conflict = false;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conflict = m_shooter.getSpeakerMode() || m_shooter.getClimbMode() || m_shooter.getIntakeMode();
    if (!conflict){
      m_shooter.setAmpMode(true);
      m_shooter.setShooterPosition(ShooterPosition.AMP);
      m_shooter.setShooterMotors(0.078,0.38);
    }
    else{
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conflict = m_shooter.getSpeakerMode() || m_shooter.getClimbMode() || m_shooter.getIntakeMode();
    if (conflict){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!conflict){
      m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
      m_shooter.setShooterMotors(0);
      m_shooter.setAmpMode(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
