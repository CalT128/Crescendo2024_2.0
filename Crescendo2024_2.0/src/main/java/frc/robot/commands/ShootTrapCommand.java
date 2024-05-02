// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootTrapCommand extends Command {
  /** Creates a new ShootTrapCommand. */
  ShooterSubsystem m_shooter;
  boolean isFinished;
  public ShootTrapCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    m_shooter.setFeedMotor(-1);
    m_shooter.setShooterMotors(-0.3);
    m_shooter.setShooterPosition(ShooterPosition.SOURCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.getStartEncoderTripped()){
      m_shooter.setFeedMotor(-0.2);
      m_shooter.setShooterMotors(0.17);
      if (m_shooter.getDistanceEncoderTripped()){
        m_shooter.setFeedMotor(0);
        m_shooter.setShooterMotors(0);
        isFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setFeedMotor(0);
    m_shooter.setShooterMotors(0);
    m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
