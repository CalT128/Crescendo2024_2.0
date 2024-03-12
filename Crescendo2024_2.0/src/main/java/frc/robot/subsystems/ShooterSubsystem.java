// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterPosition;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  //OTHER SUBSYSTEMS
  
  //MOTORS
  CANSparkMax topShooterMotor;
  CANSparkMax bottomShooterMotor;
  CANSparkMax shooterFeedMotor;
  CANSparkMax shooterAngleMotor;
  //ENCODERS
  RelativeEncoder shooterAngleEncoder;
  AnalogInput distanceEncoder;
  boolean distanceEncoderTripped;
  //SHOOTER ANGLE POSITIONS
  Map<ShooterPosition, Double> positionsMap;
  double shooterSpeakerPosition;
  double currentRotations;
  double targetRotations; 
  ShooterPosition targetPosition;
  ProfiledPIDController angleController;
  double shooterAngleEncoderCalculate;
  //MODES
  boolean speakerMode;
  boolean ampMode;
  boolean climbMode;
  boolean intakeMode;

  public ShooterSubsystem() {
    //OTHER SUBSYSTEMS
    
    //MOTORS
    topShooterMotor = new CANSparkMax(10, MotorType.kBrushless);
    topShooterMotor.setInverted(true);
    bottomShooterMotor = new CANSparkMax(11,MotorType.kBrushless);
    shooterFeedMotor = new CANSparkMax(12, MotorType.kBrushless);
    shooterAngleMotor = new CANSparkMax(13, MotorType.kBrushless);
    shooterAngleMotor.setInverted(true);
    //ENCODERS
    shooterAngleEncoder = shooterAngleMotor.getEncoder();
    distanceEncoder = new AnalogInput(0);
    distanceEncoderTripped = false;
    //SHOOTER ANGLEPOSITIONS
    shooterSpeakerPosition = Constants.ShooterConstants.shooterSpeakerPosition;
    positionsMap = new HashMap<ShooterPosition, Double>(){{
      put(ShooterPosition.INTAKE, ShooterConstants.shooterIntakePosition);
      put(ShooterPosition.AMP, ShooterConstants.shooterAmpPosition);
      put(ShooterPosition.SPEAKER, shooterSpeakerPosition);
      put(ShooterPosition.CLIMB, ShooterConstants.shooterClimbPosition);
      put(ShooterPosition.DEFAULT, ShooterConstants.shooterDefaultPosition);
    }};
    targetPosition = ShooterPosition.DEFAULT;
    angleController = new ProfiledPIDController(Constants.ShooterConstants.kP,Constants.ShooterConstants.kI,Constants.ShooterConstants. kD, new TrapezoidProfile.Constraints(1.7,999999999));
    angleController.setTolerance(0.001);
    shooterAngleEncoderCalculate = 0;
    speakerMode = false;
    ampMode = false;
    climbMode = false;
    intakeMode = false;
  }
  public void setSpeakerMode(boolean speakerMode){
    this.speakerMode = speakerMode;
  }
  public void setAmpMode(boolean ampMode){
    this.ampMode = ampMode;
  }
  public void setClimbMode(boolean climbMode){
    this.climbMode = climbMode;
  }
  public boolean getSpeakerMode(){
    return speakerMode;
  }
  public boolean getAmpMode(){
    return ampMode;
  }
  public boolean getClimbMode(){
    return climbMode;
  }
  public boolean getIntakeMode(){
    return intakeMode;
  }
  public void setIntakeMode(boolean intakeMode){
    this.intakeMode = intakeMode;
  }
  
  public boolean getDistanceEncoderTripped(){
    return distanceEncoderTripped;
  }
  public void setShooterPosition(ShooterPosition position){
    if (intakeMode){
      targetPosition = ShooterPosition.INTAKE;
    }
    else{
      targetPosition = position;
    }
  }
  public void setSpeakerPosition(double rotations){
    positionsMap.replace(ShooterPosition.SPEAKER,rotations);
    shooterSpeakerPosition = rotations;
  }
  public void setShooterMotors(double speed){
    topShooterMotor.set(speed);
    bottomShooterMotor.set(speed);
  }
  public void setShooterMotors(double topSpeed, double bottomSpeed){
    topShooterMotor.set(topSpeed);
    bottomShooterMotor.set(bottomSpeed);
  }
  public void setFeedMotor(double speed){
    shooterFeedMotor.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentRotations = shooterAngleEncoder.getPosition()/Constants.ShooterConstants.shooterAngleConstants;
    if (distanceEncoder.getVoltage()>4.7){
      distanceEncoderTripped = true;
    }
    else{
      distanceEncoderTripped = false;
    }
    if (intakeMode){
      targetPosition = ShooterPosition.INTAKE;
    }
    shooterAngleMotor.setIdleMode(IdleMode.kBrake);
    if (!(shooterAngleMotor.getOutputCurrent()>40)){
      shooterAngleEncoderCalculate = angleController.calculate(currentRotations,(double)positionsMap.get(targetPosition));
      shooterAngleMotor.set(shooterAngleEncoderCalculate);
    }
    else{
      shooterAngleMotor.set(0);
    }

    SmartDashboard.putBoolean("SPEAKER MODE",speakerMode);
    SmartDashboard.putBoolean("AMP MODE",ampMode);
    SmartDashboard.putBoolean("CLIMB MODE", climbMode); 
  }
}
