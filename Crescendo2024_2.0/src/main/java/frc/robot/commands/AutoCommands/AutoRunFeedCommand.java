// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRunFeedCommand extends Command {
  /** Creates a new AutoRunFeedCommand. */
  ShooterSubsystem m_shooter;
  Timer timer;
  boolean isFinished;
  public AutoRunFeedCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    timer = new Timer();
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
    if (!(timer.get()>1)){
      m_shooter.setFeedMotor(1);
    }
    else{
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setFeedMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
