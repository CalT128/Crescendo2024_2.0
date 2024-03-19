// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  //OTHER SUBSYTEMS
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  //MOTORS
  CANSparkMax liftMotor;
  //SOLENOIDS
  DoubleSolenoid liftSolenoid;
  //INIT SETTINGS
  boolean init;
  //SAFTEY SETTINGS
  boolean currentTripped;
  boolean toggleSolenoid;
  public ClimbSubsystem(IntakeSubsystem intake,ShooterSubsystem shooter) {
    //OTHER SUSBSYSTEMS
    m_intake = intake;
    m_shooter = shooter;
    //MOTORS
    liftMotor = new CANSparkMax(20, MotorType.kBrushless);
    //SOLENOIDS
    liftSolenoid = m_intake.getPeneumaticHub().makeDoubleSolenoid(5, 4);
    //INIT SETTINGS
    init = true;
    //SAFTEY SETTINGS
    currentTripped = false;
    toggleSolenoid = false;
  }
  public void lift(){
    if (!currentTripped){
      liftMotor.set(1);
    }
    
  }
  public void retract(){
    if (!currentTripped){
      liftMotor.set(-1);
    }
    
  }
  public void stop(){
    liftMotor.set(0);
  }
  public void toggleSolenoid(){
    //liftSolenoid.set(DoubleSolenoid.Value.kForward);
    toggleSolenoid = !toggleSolenoid;
    if (toggleSolenoid){
      liftSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else{
      liftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    liftMotor.setIdleMode(IdleMode.kBrake);            
    if (liftMotor.getOutputCurrent()>200){
      currentTripped = true;
      liftMotor.set(0);
    }
    else{
      currentTripped = false;
    }
    if (init){
      liftSolenoid.set(DoubleSolenoid.Value.kReverse);
      init = false;
    }

  }
}
