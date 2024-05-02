// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DeployIntakeCommand extends Command {
  /** Creates a new DeployIntakeCommand. */
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  boolean isFinished;
  public DeployIntakeCommand(IntakeSubsystem intake,ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_shooter = shooter;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    m_intake.setIntakeSequenceFinished(false);
    m_shooter.setDistanceEncoder(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.deploySolenoidSequence(true);
    
    //System.out.println("Hello");
    //System.out.println(m_shooter.getDistanceEncoderTripped());
    if (m_shooter.getDistanceEncoderTripped()){
      
      //m_intake.getDriverJoystick().setRumble(RumbleType.kBothRumble,0.4);
      //m_intake.getOperatorJoystick().setRumble(RumbleType.kBothRumble,0.4);
      //System.out.println("Hello");
      m_shooter.setFeedMotor(0);
      isFinished = true;
    }
    if (m_shooter.getStartEncoderTripped()){
      m_intake.setRumble(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setFeedMotor(0);
    m_intake.disableIntake();
    //m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //m_intake.getDriverJoystick().setRumble(RumbleType.kBothRumble,0);
    //m_intake.getOperatorJoystick().setRumble(RumbleType.kBothRumble,0);
    return isFinished;
  }
}
