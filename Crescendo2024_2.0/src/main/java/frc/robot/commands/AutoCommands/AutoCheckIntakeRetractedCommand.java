// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCheckIntakeRetractedCommand extends Command {
  /** Creates a new AutoCheckIntakeRetractedCommand. */
  IntakeSubsystem m_intake;
  Timer timer;
  boolean isFinished;
  public AutoCheckIntakeRetractedCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    m_intake = intake;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_intake.getAutoIntakeMode()){
      isFinished = true;
    }
    if (timer.get()>1.5){
      isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setAutoIntakeMode(false);
    m_intake.disableIntake();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
