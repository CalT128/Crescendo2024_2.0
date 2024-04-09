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

import edu.wpi.first.math.controller.PIDController;
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
  SwerveSubsystem m_swerve;
  //MOTORS
  CANSparkMax topShooterMotor;
  CANSparkMax bottomShooterMotor;
  CANSparkMax shooterFeedMotor;
  CANSparkMax shooterAngleMotor;
  //ENCODERS
  RelativeEncoder shooterAngleEncoder;
  AnalogInput distanceEncoder;
  AnalogInput startEncoder;
  boolean distanceEncoderTripped;
  //SHOOTER ANGLE POSITIONS
  Map<ShooterPosition, Double> positionsMap;
  double shooterSpeakerPosition;
  double currentRotations;
  double targetRotations; 
  ShooterPosition targetPosition;
  ProfiledPIDController angleController;
  PIDController shooterMotorController;
  double shooterAngleEncoderCalculate;
  //MODES
  boolean speakerMode;
  boolean ampMode;
  boolean climbMode;
  boolean intakeMode;
  boolean init;
  double velocitySpeedTop;
  double velocitySpeedBottom;
  boolean starterSensorTripped;

  public ShooterSubsystem(SwerveSubsystem swerve) {
    velocitySpeedTop = 0;
    velocitySpeedBottom = 0;
    init = true;
    //OTHER SUBSYSTEMS
    m_swerve = swerve;
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
    startEncoder = new AnalogInput(1);
    starterSensorTripped = false;
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
    angleController = new ProfiledPIDController(Constants.ShooterConstants.kP,Constants.ShooterConstants.kI,Constants.ShooterConstants. kD, new TrapezoidProfile.Constraints(2,999999999));
    angleController.setTolerance(0.001);
    shooterMotorController = new PIDController(0.00000645, 0.00000001, 0.0000000);
    //shooterMotorController = new PIDController()
    shooterMotorController.setTolerance(5);
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
  public void resetVelocity(){
    velocitySpeedTop = 0;
    velocitySpeedBottom = 0;
  }
  public void setIntakeMode(boolean intakeMode){
    this.intakeMode = intakeMode;
  }
  
  public boolean getDistanceEncoderTripped(){
    return distanceEncoderTripped;
  }
  public void setShooterPosition(ShooterPosition position){
    targetPosition = position;
  }
  public void setSpeakerPosition(double rotations){
    if (rotations<0){
      rotations = 0;
    }
    positionsMap.replace(ShooterPosition.SPEAKER,rotations);
    shooterSpeakerPosition = rotations;
  }
  
  public void setShooterMotorVelocity(double velocityTop, double velocityBottom){
    double topCurrent = topShooterMotor.getEncoder().getVelocity();
    double bottomCurrent = bottomShooterMotor.getEncoder().getVelocity();
    velocitySpeedTop += shooterMotorController.calculate(topCurrent,velocityTop);
    velocitySpeedBottom += shooterMotorController.calculate(bottomCurrent,velocityBottom);
    topShooterMotor.set(velocitySpeedTop);
    //System.out.println(shooterMotorController.calculate(topCurrent,velocityTop));
    //System.out.println(velocitySpeedTop);
    bottomShooterMotor.set(velocitySpeedBottom);
  }
  public void setShooterMotors(double speed){
    topShooterMotor.set(speed);
    bottomShooterMotor.set(speed);
  }
  public void setShooterMotors(double topSpeed, double bottomSpeed){
    topShooterMotor.set(topSpeed);
    bottomShooterMotor.set(bottomSpeed);
  }
  public void setDistanceEncoder(boolean distanceEncoder){
    distanceEncoderTripped = distanceEncoder;
  }
  public void setFeedMotor(double speed){
    shooterFeedMotor.set(speed);
  }
  public double getCurrentRotations(){
    return currentRotations;
  }
  public boolean getStartEncoderTripped(){
    return starterSensorTripped;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("top velocity",topShooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("bottom velocity",bottomShooterMotor.getEncoder().getVelocity());
    if (init){
      topShooterMotor.setIdleMode(IdleMode.kBrake);
      bottomShooterMotor.setIdleMode(IdleMode.kBrake);
      init = false;
    }
    // This method will be called once per scheduler run
    currentRotations = shooterAngleEncoder.getPosition()/Constants.ShooterConstants.shooterAngleConstants;
    //System.out.println(startEncoder.getVoltage());
    //System.out.println(distanceEncoder.getVoltage());
    if (distanceEncoder.getVoltage()<4.5){
      distanceEncoderTripped = false;
    }
    else{
      distanceEncoderTripped = true;
    }
    if (startEncoder.getVoltage()<4.5){
      starterSensorTripped = false;

    }
    else{
      
      starterSensorTripped = true;
    }
    //System.out.println(distanceEncoderTripped);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake);
    if (!(shooterAngleMotor.getOutputCurrent()>60)){
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
