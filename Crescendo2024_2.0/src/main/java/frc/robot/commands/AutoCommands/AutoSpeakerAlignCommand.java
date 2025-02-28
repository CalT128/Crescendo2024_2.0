// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoSpeakerAlignCommand extends Command {
  /** Creates a new AutoSpeakerAlignCommand. */
  SwerveSubsystem m_swerve;
  ShooterSubsystem m_shooter;
  PhotonVisionSubsystem m_vision;
  Timer timer;
  boolean isFinished;
  boolean align;
  public AutoSpeakerAlignCommand(SwerveSubsystem m_swerve, ShooterSubsystem shooter, PhotonVisionSubsystem vision,boolean align) {
    // Use addRequirements() here to declare subsystem dependencies.
    this. m_swerve = m_swerve;
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
    m_shooter.setShooterMotors(0.9);
    timer.start();
    if (!align){
      isFinished = true;
      m_vision.resetIsAligned();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.getIsXAligned() && m_vision.getIsYAligned()){
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
      m_shooter.setShooterMotors(0);
    }
    m_vision.resetIsAligned();
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
