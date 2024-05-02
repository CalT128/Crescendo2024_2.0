// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeakerCommand extends Command {
  /** Creates a new ShootSpeaker. */
  ShooterSubsystem m_shooter;
  PhotonVisionSubsystem m_vision;
  boolean isFinished;
  
  public ShootSpeakerCommand(ShooterSubsystem shooter,PhotonVisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_shooter = shooter;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    if (m_shooter.getSpeakerMode()){
      /*if (m_vision.getIsYAligned() && m_vision.getIsXAligned()){
        m_shooter.setFeedMotor(1);
      }
      else{
        m_shooter.setFeedMotor(0);
      }*/
      m_shooter.setFeedMotor(1);
      
    }
    else{
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (m_vision.getIsYAligned() && m_vision.getIsXAligned()){
      m_shooter.setFeedMotor(1);
    }
    else{
      m_shooter.setFeedMotor(0);
    }*/
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_shooter.getSpeakerMode()){
      m_shooter.setFeedMotor(0);
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
