// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterPosition;



public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  SwerveSubsystem m_swerve;
  ShooterSubsystem m_shooter;
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
  boolean autoIntakeMode;
  //starting positions
  boolean init;
  Joystick driverJoystick;
  Joystick operatorJoystick;
  boolean rumble;
  Timer rumbleTimer;
  boolean disableShooterIntake;
  DoubleSolenoid placeHolder;

  
  public IntakeSubsystem(SwerveSubsystem swerve,ShooterSubsystem shooter,Joystick driver, Joystick operator) {
    disableShooterIntake = false;
   driverJoystick = driver;
   operatorJoystick = operator;
    m_swerve = swerve;
    m_shooter = shooter;
    rumble = false;
    rumbleTimer = new Timer();
    //MOTORS
    intakeMotor = new CANSparkMax(9,MotorType.kBrushless);
    //PNEUMATICS
    pneumaticHub = new PneumaticHub();
    pneumaticHub.enableCompressorAnalog(90,100);
    topSolenoid = pneumaticHub.makeDoubleSolenoid(1,0);
    bottomSolenoid = pneumaticHub.makeDoubleSolenoid(3,2);
    placeHolder = pneumaticHub.makeDoubleSolenoid(7, 6);
    //TIMER
    timer = new Timer();
    //MODES
    intakeToggled = false;
    intakeSequenceFinished = true;
    intakeMode = false;
    disableIntake = false;
    //starting position
    init = true;
    autoIntakeMode = false;
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
  public  boolean getIntakeSequenceFinished(){
    return intakeSequenceFinished;
  }
  public void setBottomSolenoid(){
    bottomSolenoid(DoubleSolenoid.Value.kForward);
  }
  public void setTopSolenoid(){
    topSolenoid(DoubleSolenoid.Value.kForward);
  }
  public void resetTopSolenoid(){
    topSolenoid(DoubleSolenoid.Value.kReverse);
  }
  public void deployShooterSequence(boolean deploy){
    DoubleSolenoid.Value solenoidValue;
    if (!intakeSequenceFinished){
      timer.start();
      if (deploy){
        solenoidValue = DoubleSolenoid.Value.kForward;
        bottomSolenoid.set(solenoidValue);
        if (timer.get()>0.3){
          topSolenoid.set(solenoidValue);
          intakeSequenceFinished = true;
        }
      }
      else{
        solenoidValue = DoubleSolenoid.Value.kReverse;  
        topSolenoid.set(solenoidValue);
        if (timer.get()>0.3){
          bottomSolenoid.set(solenoidValue);
          intakeSequenceFinished = true;
          disableShooterIntake = false;
        }
      }
    }
  }
  public void disableShooterIntake(){
    disableShooterIntake = true;
  }
  public void deploySolenoidSequence(boolean deploy){
    DoubleSolenoid.Value solenoidValue;
    if (intakeMode){
      if (!m_shooter.getStartEncoderTripped()){
        m_shooter.setFeedMotor(1);
      }
      else{
        System.out.println("hhhh");
        m_shooter.setFeedMotor(0.4);
      }
    }
    if (!intakeSequenceFinished){
      timer.start();
      if (deploy){
        //m_shooter.setFeedMotor(1);
        
        intakeMode = true;
        
        if (!m_shooter.getClimbMode()){
          runIntakeMotor(1);
          
          m_shooter.setShooterPosition(ShooterPosition.INTAKE);
          
        }
        
        solenoidValue = DoubleSolenoid.Value.kForward;
        bottomSolenoid.set(solenoidValue);
        if (timer.get()>0.3){
          topSolenoid.set(solenoidValue);
          intakeSequenceFinished = true;
        }
      }
      else {
        //System.out.println("Hello");
        //m_shooter.setFeedMotor(0);
        intakeMode = false;
        m_shooter.setShooterPosition(ShooterPosition.DEFAULT);
        runIntakeMotor(0);
        solenoidValue = DoubleSolenoid.Value.kReverse;
        topSolenoid.set(solenoidValue);
        if (timer.get()>0.2 && timer.get()<0.4){
          m_shooter.setFeedMotor(-0.072);
        }
        else{
          m_shooter.setFeedMotor(0);
        }
        if (timer.get()>0.8){
          m_shooter.setFeedMotor(0);
          bottomSolenoid.set(solenoidValue);
          intakeSequenceFinished = true;
          timer.reset();
          timer.stop();
          disableIntake = false;
          
        }
      }
      //System.out.println("HELLO");
    }
    else{
      //intakeMode = false;
      //System.out.println("hello");
      timer.reset();
      timer.stop();
    }
    
  }
  public Joystick getDriverJoystick(){
    return driverJoystick;
  }
  public Joystick getOperatorJoystick(){
    return operatorJoystick;
  }
  public void setAutoIntakeMode(boolean autoIntake){
    this.autoIntakeMode = autoIntake;
  }
  public boolean getAutoIntakeMode(){
    return autoIntakeMode;
  }
  public void setRumble(boolean rumble){
    this.rumble = rumble;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    placeHolder.set(DoubleSolenoid.Value.kReverse);
    if (rumble){
      rumbleTimer.start();
      driverJoystick.setRumble(RumbleType.kBothRumble,0.5);
      operatorJoystick.setRumble(RumbleType.kBothRumble,0.5);
      if (rumbleTimer.get()>1){
        driverJoystick.setRumble(RumbleType.kBothRumble,0);
        operatorJoystick.setRumble(RumbleType.kBothRumble,0);
        rumble = false;
        rumbleTimer.reset();
        rumbleTimer.stop();
      }
    }
    if (autoIntakeMode){
      deploySolenoidSequence(true);
      if (m_shooter.getDistanceEncoderTripped()){
        //System.out.println("Encoder tripped disable intake");
        setIntakeSequenceFinished(false);
        disableIntake();
        autoIntakeMode = false;
      }
    }
    m_shooter.setIntakeMode(intakeMode);
    if (init){
      topSolenoid.set(DoubleSolenoid.Value.kReverse);
      bottomSolenoid.set(DoubleSolenoid.Value.kReverse);
      init = false;
    }
    if (disableIntake){
      //m_shooter.setFeedMotor(0);
      setIntakeSequenceFinished(false);
      deploySolenoidSequence(false);
      //System.out.println("Hello");
    }
    if (disableShooterIntake){
      setIntakeSequenceFinished(false);
      deployShooterSequence(false);
    }
    SmartDashboard.putBoolean("INTAKE MODE:",intakeMode);
  }
}
