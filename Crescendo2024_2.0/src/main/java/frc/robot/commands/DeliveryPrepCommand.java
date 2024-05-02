// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DeliveryPrepCommand extends Command {
  /** Creates a new DeliveryPrepCommand. */
  ShooterSubsystem m_shooter;
  SwerveSubsystem m_swerve;
  boolean isFinished;
  boolean conflict;
  public DeliveryPrepCommand(SwerveSubsystem swerve,ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_shooter = shooter;
    isFinished = false;
    conflict = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    conflict = m_shooter.getAmpMode() || m_shooter.getClimbMode() || m_shooter.getIntakeMode();
    if (!conflict){
      m_shooter.setSpeakerMode(true);
      m_shooter.setShooterPosition(ShooterPosition.SPEAKER);
      m_swerve.setDriveDeliveryPrep(true);
      m_shooter.setSpeakerPosition(0.4);
      m_shooter.setShooterMotors(0.63);
    }
    else{
      isFinished = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!conflict){
      m_swerve.setDriveDeliveryPrep(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!conflict){
      m_swerve.setDriveDeliveryPrep(false);
      m_shooter.setSpeakerMode(false);
      m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
      m_shooter.setShooterMotors(0);
    }
    m_swerve.setLockedOn(false);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
