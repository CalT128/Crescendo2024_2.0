// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetAutoIntakeModeCommand extends InstantCommand {
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  boolean autoIntakeMode;
  public SetAutoIntakeModeCommand(IntakeSubsystem intake,ShooterSubsystem shooter,boolean autoIntakeMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_shooter = shooter;
    this.autoIntakeMode = autoIntakeMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setAutoIntakeMode(autoIntakeMode);
    if (!autoIntakeMode){
      m_shooter.setFeedMotor(0);
      m_shooter.setShooterPosition(ShooterPosition.DEFAULT);

    }
  }
}
