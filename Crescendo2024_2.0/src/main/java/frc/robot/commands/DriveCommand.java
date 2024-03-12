// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vector;

public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  Joystick driverJoystick;
  SwerveSubsystem m_swerve;
  Vector v;
  double xRStick;
  double yRStick;
  double strafeMagnatude;
  double strafeTargetDegree;
  double rotationMagnatude;
  double degreeOffset;
  boolean joystickOff;

  public DriveCommand(SwerveSubsystem swerve, Joystick stick) {
    m_swerve = swerve;
    driverJoystick = stick;
    joystickOff = false;
    addRequirements(m_swerve);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    strafeMagnatude = driverJoystick.getMagnitude();
    strafeTargetDegree = driverJoystick.getDirectionDegrees();
    //double rotationMagnatude = -driverJoystick.getRawAxis(4);
    xRStick = driverJoystick.getRawAxis(4);
    yRStick = -driverJoystick.getRawAxis(5);
    //System.out.println(y);
    // double magnatude = Math.abs(Math.sqrt(Math.pow(x,2) + Math.pow(y,2)));
    v = new Vector(xRStick,yRStick);
    degreeOffset = m_swerve.getDegreeOffset();
   
    rotationMagnatude = m_swerve.driveToDegree(degreeOffset,v.getDegree());
    if (xRStick==0 && yRStick == 0){
      rotationMagnatude = 0;
    }
    if (strafeMagnatude == 0 && (xRStick == 0 && yRStick == 0)){
      joystickOff = true;
    }
    else{
      joystickOff = false;
    }
    m_swerve.setJoystickOff(joystickOff);
    strafeTargetDegree = (strafeTargetDegree + 360) % 360;
    strafeTargetDegree = 360 - strafeTargetDegree;
    //strafeMagnatude = 0.1;
    //strafeTargetDegree = 0;
    //rotationMagnatude = 0;
    m_swerve.drive(strafeMagnatude, strafeTargetDegree, rotationMagnatude);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
