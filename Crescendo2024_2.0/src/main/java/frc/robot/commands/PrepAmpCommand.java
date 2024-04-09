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
    isFinished = false;
    conflict = m_shooter.getSpeakerMode() || m_shooter.getClimbMode() || m_shooter.getIntakeMode();
    //System.out.println(conflict);
    if (!conflict){

      m_shooter.setAmpMode(true);
      m_shooter.setShooterPosition(ShooterPosition.AMP);
      m_shooter.resetVelocity();
      //m_shooter.setShooterMotorVelocity(0.082,0.44);
      
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
    //m_shooter.setShooterMotorVelocity(580,1995);// 1 2 and 3 amp shot
    //m_shooter.setShooterMotorVelocity(600,2000);//GOOD ONE FOR SOFT
    //m_shooter.setShooterMotorVelocity(640, 2050);
    m_shooter.setShooterMotorVelocity(360,2400);
    
    //m_shooter.setShooterMotorVelocity(540,1930);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setAmpMode(false);
    if (!conflict){
      m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
      m_shooter.setShooterMotors(0);
      
    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
