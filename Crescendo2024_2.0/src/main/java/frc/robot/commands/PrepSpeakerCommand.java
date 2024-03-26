// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepSpeakerCommand extends Command {
  /** Creates a new PrepSpeakerCommand. */
  ShooterSubsystem m_shooter;
  PhotonVisionSubsystem m_vision;
  boolean isFinished;
  boolean conflict;

  public PrepSpeakerCommand(ShooterSubsystem shooter,PhotonVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_vision = vision;
    isFinished = false;
    conflict = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    conflict = m_shooter.getAmpMode() || m_shooter.getClimbMode() || m_shooter.getIntakeMode();
    //System.out.println(conflict);
    if (!conflict){
      
      m_shooter.setSpeakerMode(true);
      m_shooter.setShooterPosition(ShooterPosition.SPEAKER);
      m_shooter.setShooterMotors(0.87);
    }
    else{
      isFinished = true;
      
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conflict = m_shooter.getAmpMode() || m_shooter.getClimbMode() || m_shooter.getIntakeMode();
    //System.out.println(conflict);
    if (!conflict){
      m_vision.ameliorateX();
      m_vision.correctLauncher();
    }
    else{
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setSpeakerMode(false);
    if (!conflict){
      m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
      m_shooter.setShooterMotors(0);
      m_vision.setLockedOn(false);//CHECK LOCKEDON
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
