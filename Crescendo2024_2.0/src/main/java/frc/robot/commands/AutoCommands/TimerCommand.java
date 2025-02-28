// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TimerCommand extends Command {
  /** Creates a new TimerCommand. */
  Timer timer;
  double time;
  boolean isFinished;
  
  public TimerCommand(double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.time
     = time;
    isFinished = false;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Start TIMER");
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get()>time){
      isFinished = true;
      //System.out.println("Finished TIMER");
      timer.stop();
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
