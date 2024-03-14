// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpeakerAlignCommand extends Command {
  /** Creates a new AutoSpeakerAlignCommand. */
  ShooterSubsystem m_shooter;
  PhotonVisionSubsystem m_vision;
  Timer timer;
  boolean isFinished;
  boolean align;
  public AutoSpeakerAlignCommand(ShooterSubsystem shooter, PhotonVisionSubsystem vision,boolean align) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_vision = vision;
    timer = new Timer();
    isFinished = false;
    this.align = align;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setAutoAlign(align);
    timer.start();
    if (align){
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()>0.7){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!align){
      m_vision.setLockedOn(false);
      m_vision.setAutoAlign(false);
      m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
      
    }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
