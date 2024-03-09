// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepClimbCommand extends Command {
  /** Creates a new PrepClimbCommand. */
  ShooterSubsystem m_shooter;
  boolean conflict;
  boolean isFinished;
  public PrepClimbCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    conflict = false;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conflict = m_shooter.getAmpMode() || m_shooter.getSpeakerMode() || m_shooter.getIntakeMode();
    if (conflict){
      isFinished = true;
    }
    else{
      m_shooter.setClimbMode(true);
      m_shooter.setShooterPosition(ShooterPosition.CLIMB);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!conflict){
      m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
      m_shooter.setClimbMode(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
