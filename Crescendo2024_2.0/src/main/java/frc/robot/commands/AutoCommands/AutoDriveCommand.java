// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveCommand extends Command {
  /** Creates a new AutoDriveCommand. */
  SwerveSubsystem m_swerve;
  double xfeet;
  double yfeet;
  double rotation;
  boolean inBetween;
  boolean isFinished;
  double mult;
  public AutoDriveCommand(SwerveSubsystem swerve,double x, double y, double rotation, boolean inBetween, double mult) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    xfeet = x;
    yfeet = y;
    this.rotation = rotation;
    this.inBetween = inBetween;
    isFinished = false;
    this.mult = mult;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.setAutoPos();
    m_swerve.setAutoAtSetPoint(false);
    m_swerve.prepInbetweenPoint(xfeet,yfeet);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.autoDrive(xfeet,yfeet,rotation,inBetween,mult);
    if (m_swerve.getAtAutoSetPoint()){
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
