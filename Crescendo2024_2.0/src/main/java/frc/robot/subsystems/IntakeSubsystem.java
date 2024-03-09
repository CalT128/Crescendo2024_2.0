// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  //MOTORS
  CANSparkMax intakeMotor;
  //PNEUMATICS
  PneumaticHub pneumaticHub;
  DoubleSolenoid topSolenoid;
  DoubleSolenoid bottomSolenoid;
  //TIMER
  Timer timer;
  //MODES
  boolean intakeToggled;
  boolean intakeSequenceFinished;
  boolean intakeMode;
  boolean disableIntake;
  //starting positions
  boolean init;

  
  public IntakeSubsystem() {
    //MOTORS
    intakeMotor = new CANSparkMax(9,MotorType.kBrushless);
    //PNEUMATICS
    pneumaticHub = new PneumaticHub();
    pneumaticHub.enableCompressorAnalog(90,100);
    topSolenoid = pneumaticHub.makeDoubleSolenoid(1,0);
    bottomSolenoid = pneumaticHub.makeDoubleSolenoid(3,2);
    //TIMER
    timer = new Timer();
    //MODES
    intakeToggled = false;
    intakeSequenceFinished = true;
    intakeMode = false;
    disableIntake = false;
    //starting position
    init = true;
  }
  public PneumaticHub getPeneumaticHub(){
    return pneumaticHub;
  }
  public boolean getIntakeMode(){
    return intakeMode;
  }
  public void runIntakeMotor(double speed){
    intakeMotor.set(speed);
  }
  public void stopIntakeMotor(){
    intakeMotor.set(0);
  }
  public void topSolenoid(DoubleSolenoid.Value value){
    topSolenoid.set(value);
  }
  public void bottomSolenoid(DoubleSolenoid.Value value){
    bottomSolenoid.set(value);
  }
  public void disableIntake(){
    disableIntake = true;
  }
  public void setIntakeSequenceFinished(boolean isFinished){
    intakeSequenceFinished = isFinished;
  }
  public void deploySolenoidSequence(boolean deploy){
    DoubleSolenoid.Value solenoidValue;
    if (!intakeSequenceFinished){
      intakeMode = true;
      timer.start();
      if (deploy){
        runIntakeMotor(1);
        solenoidValue = DoubleSolenoid.Value.kForward;
        bottomSolenoid.set(solenoidValue);
        if (timer.get()>0.3){
          topSolenoid.set(solenoidValue);
          intakeSequenceFinished = true;
        }
      }
      else {
        intakeMode = false;
        runIntakeMotor(0);
        solenoidValue = DoubleSolenoid.Value.kReverse;
        topSolenoid.set(solenoidValue);
        if (timer.get()>0.8){
          bottomSolenoid.set(solenoidValue);
          intakeSequenceFinished = true;
          disableIntake = false;
        }
      }
    }
    else{
      intakeMode = false;
      timer.reset();
      timer.stop();
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (init){
      topSolenoid.set(DoubleSolenoid.Value.kReverse);
      bottomSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    if (disableIntake){
      setIntakeSequenceFinished(false);
      deploySolenoidSequence(false);
    }
    SmartDashboard.putBoolean("INTAKE MODE:",intakeMode);
    
  }
}
