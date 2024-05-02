// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmpCommand extends Command {
  /** Creates a new ShootAmpCommand. */
  ShooterSubsystem m_shooter;
  Timer timer;
  boolean isFinished;
  public ShootAmpCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (!m_shooter.getAmpMode()){
      isFinished = true;
    }
    else{
      m_shooter.setFeedMotor(0.7);
      m_shooter.setShooterPosition(ShooterPosition.SOURCE);//CHECK
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_shooter.getAmpMode()){
      m_shooter.setFeedMotor(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
