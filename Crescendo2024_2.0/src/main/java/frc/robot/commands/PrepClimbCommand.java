// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepClimbCommand extends Command {
  /** Creates a new PrepClimbCommand. */
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  ClimbSubsystem m_climb;
  boolean conflict;
  boolean isFinished;
  public PrepClimbCommand(ShooterSubsystem shooter,IntakeSubsystem intake,ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_climb = climb;
    m_intake = intake;
    conflict = false;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    conflict = m_shooter.getAmpMode() || m_shooter.getSpeakerMode() || m_shooter.getIntakeMode();
    if (conflict){
      isFinished = true;
    }
    else{
      m_climb.toggleSolenoid();
      m_shooter.setClimbMode(true);
      m_shooter.setShooterPosition(ShooterPosition.CLIMB);
      m_intake.setBottomSolenoid();
      //m_intake.setIntakeSequenceFinished(false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_intake.deploySolenoidSequence(true);
  }

  // Called once the command ends or is interrupte
  
  @Override
  public void end(boolean interrupted) {
    //m_shooter.setClimbMode(false);
    //m_shooter.setShooterPosition(ShooterPosition.CLIMB2);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
