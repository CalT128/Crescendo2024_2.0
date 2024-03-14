// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutointakeCommand extends Command {
  /** Creates a new AutointakeCommand. */
  IntakeSubsystem m_intake;
  boolean isFinished;
  public AutointakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    isFinished = false;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeSequenceFinished(false);
    m_intake.deploySolenoidSequence(true);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getIntakeSequenceFinished()){
      isFinished = true;
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
