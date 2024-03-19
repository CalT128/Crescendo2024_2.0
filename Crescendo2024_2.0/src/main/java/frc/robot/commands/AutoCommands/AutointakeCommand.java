// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {
  /** Creates a new AutointakeCommand. */
  IntakeSubsystem m_intake;
  boolean isFinished;
  boolean deploy;
  public AutoIntakeCommand(IntakeSubsystem intake,boolean deploy) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    isFinished = false;
    this.deploy = deploy;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeSequenceFinished(false);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (deploy){
      m_intake.setAutoIntakeMode(true);
    }
    else{
      m_intake.setAutoIntakeMode(false);
      m_intake.deploySolenoidSequence(false);
    }
    if (m_intake.getIntakeSequenceFinished()){
        isFinished = true;
        //System.out.println("Hello Hello");
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
