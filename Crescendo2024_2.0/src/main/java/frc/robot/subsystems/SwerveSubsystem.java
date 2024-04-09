// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import org.photonvision.PhotonCamera;

//import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.revrobotics.TalonFXLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  TalonFX frontLeftDriver;
  TalonFX frontLeftTurner;
  TalonFX frontRightDriver;
  TalonFX frontRightTurner;
  TalonFX backLeftDriver;
  TalonFX backLeftTurner;
  TalonFX backRightDriver;
  TalonFX backRightTurner;

  CANcoder fLSensor;
  CANcoder fRSensor;
  CANcoder bLSensor;
  CANcoder bRSensor;

  Pigeon2 pigeon;

  double strafeMagnatude;
  double strafeDirection;

  double rotationalMagnatude;
  double rotationalDirection;

  double combinedMagnatude;
  double combinedDirection;
  
  double targetRobotDegree;

  double originalDegree;
  double degreeOffset;
  
  boolean once;
  boolean instance;
  
  PIDController directionCorrector;
  PIDController rotateToDegreeController;
  PIDController autoRotateToDegreeController;
  ProfiledPIDController yDisplacementController;
  ProfiledPIDController xDisplacementController;

  double yDisplacement;
  double xDisplacement;
  double yAcceleration;
  double xAcceleration;
  double directionCorrectorValue;

  Vector frontLeftVector;
  Vector frontRightVector;
  Vector backLeftVector;
  Vector backRightVector;
  Vector driveVector;

  boolean isRotating;
  double originalInstance;
  Timer timer;
  double lastTime;
  double currentTime;
  double xiVelocity;
  double xfVelocity;
  double yiVelocity;
  double yfvelocity;
  double dx;
  double dy;
  boolean highSpeed;
  double strafeMultiplier;
  SwerveOdometer odometer;
  boolean joystickOff;
  double flDifferenceTick;
  double frDifferenceTick;
  double blDifferenceTick;
  double brDifferenceTick;
  double flPreviousTick;
  double frPreviousTick;
  double blPreviousTick;
  double brPreviousTick;
  double strafeMult;
  double rotationMult;
  boolean lockedOn;
  double lockedOnRotationalMagnatude;
  boolean autoAtSetPoint;
  boolean autoMode;
  double originDX;
  double originDY;
  PhotonCamera m;
  double xCalculate;
  double yCalculate;
  double ampRobotDegree;
  boolean ampMode;
  boolean redSide;
  
  
  

  public SwerveSubsystem(){
    redSide = false;
    ampMode = false;
    ampRobotDegree = 0;
    xCalculate = 0;
    yCalculate = 0;
    
    autoMode = false;
    originDX = 0;
    originDY = 0;
    
    driveVector = new Vector(0,0);
    strafeMult = 0.7;
    rotationMult = 0.3;
    lockedOn = false;
    lockedOnRotationalMagnatude = 0;
    autoAtSetPoint = false;

    //Change device ID to the normal orientation
    frontLeftDriver = new TalonFX(2,"Drivetrain");
    frontLeftTurner = new TalonFX(1,"Drivetrain");
    frontRightDriver = new TalonFX(4,"Drivetrain");
    frontRightTurner = new TalonFX(3,"Drivetrain");
    backLeftDriver = new TalonFX(8,"Drivetrain");
    backLeftTurner = new TalonFX(7,"Drivetrain");
    backRightDriver = new TalonFX(6,"Drivetrain");
    backRightTurner = new TalonFX(5,"Drivetrain");
    directionCorrector = new PIDController(Constants.SwerveDriveConstants.rKP,Constants.SwerveDriveConstants.rKI,Constants.SwerveDriveConstants.rKD);
    rotateToDegreeController = new PIDController(Constants.SwerveDriveConstants.rKP,Constants.SwerveDriveConstants.rKI,Constants.SwerveDriveConstants.rKD);
    directionCorrector.enableContinuousInput(0, 360);
    directionCorrector.setTolerance(0.01);
    rotateToDegreeController.enableContinuousInput(0,360);
    rotateToDegreeController.setTolerance(0.009);
    yDisplacementController = new ProfiledPIDController(1.45,0.07,0.03,new TrapezoidProfile.Constraints(11,7));
    xDisplacementController = new ProfiledPIDController(1.45,0.07,0.03, new TrapezoidProfile.Constraints(11,7));
    yDisplacementController.setTolerance(0.05);
    xDisplacementController.setTolerance(0.05);
    autoRotateToDegreeController = new PIDController(Constants.SwerveDriveConstants.akP, Constants.SwerveDriveConstants.akI, Constants.SwerveDriveConstants.akD);
    autoRotateToDegreeController.setTolerance(0.009);
    autoRotateToDegreeController.enableContinuousInput(0,360);
    
    yDisplacement = 0;
    xDisplacement = 0;
    yAcceleration = 0;
    xAcceleration = 0;
    timer = new Timer();
    xiVelocity = 0;
    xfVelocity = 0;
    strafeMultiplier = 0;
    flDifferenceTick = 0;
    frDifferenceTick = 0;
    blDifferenceTick = 0;
    brDifferenceTick = 0;
    flPreviousTick = 0;
    frPreviousTick = 0;
    blPreviousTick = 0;
    brPreviousTick = 0;
    instance = true;
    directionCorrectorValue = 0;
    fLSensor = new CANcoder(24,"Drivetrain");//24
    fRSensor = new CANcoder(21,"Drivetrain");//21
    bLSensor = new CANcoder(23,"Drivetrain");//23
    bRSensor = new CANcoder(22,"Drivetrain");//22
    frontLeftModule = new SwerveModule(frontLeftDriver, frontLeftTurner,fLSensor);
    frontRightModule = new SwerveModule(frontRightDriver, frontRightTurner,fRSensor);
    backLeftModule = new SwerveModule(backLeftDriver, backLeftTurner,bLSensor);
    backRightModule = new SwerveModule(backRightDriver, backRightTurner,bRSensor);
    pigeon = new Pigeon2(25,"Drivetrain");//SET RIGHT DEVICE NUMBER
    once = true;
    strafeMagnatude = 0;

    strafeDirection = 0;
    highSpeed = false;

    combinedMagnatude = 0;
    combinedDirection = 0;
    originalDegree = 0;
    degreeOffset = 0;
    isRotating = true;
    targetRobotDegree = 0;
    originalDegree = 0;
    lastTime = 0;
    currentTime = 0;
    dx = 0;
    dy = 0;
    odometer = new SwerveOdometer();
    frontLeftDriver.setPosition(0);
    frontRightDriver.setPosition(0);
    backLeftDriver.setPosition(0);
    backRightDriver.setPosition(0);
  }
  public void setAmpMode(boolean ampMode){
    this.ampMode = ampMode;
  }
  public void drive(double strafeMagnatude1,double strafeDirection1,double rotationalMagnatude1){
    this.strafeMagnatude = strafeMagnatude1; //* strafeMult;
    this.strafeDirection = strafeDirection1 - degreeOffset;
    strafeDirection = ((strafeDirection % 360) + 360) % 360;
    if (lockedOn){
      //System.out.println("LOCKED ON TRUE");
      rotationalMagnatude = lockedOnRotationalMagnatude;
    }
    else if (ampMode){
      rotationalMagnatude = driveToDegree(getDegreeOffset(),ampRobotDegree);
    }
    else if (rotationalMagnatude1 != 0){
      isRotating = true;
      this.rotationalMagnatude = rotationalMagnatude1;
    }
    else{
      isRotating = false;
      rotationalMagnatude = directionCorrectorValue;
    }
    if (!autoMode){
      if (Math.abs(rotationalMagnatude)>0.5){
        strafeMagnatude *= 0.5;
        if (rotationalMagnatude < 0){
          rotationalMagnatude = -0.5;
        }
        else{
          rotationalMagnatude = 0.5;
        }
      }
      else{
        strafeMagnatude *= Math.abs((1-Math.abs(rotationalMagnatude)));
      }
    }
    else{
      strafeMagnatude *= 0.5;
      rotationalMagnatude *= 0.44;
    }
    //System.out.println("rotationalMagnautde " + rotationalMagnatude);

    
    driveVector = new Vector(strafeMagnatude,strafeDirection, true);
    //Rotational Vectors
    if (rotationalMagnatude<0){
      rotationalMagnatude = Math.abs(rotationalMagnatude);
      frontLeftVector = new Vector(rotationalMagnatude,(135+180) % 360,true);
      frontRightVector = new Vector(rotationalMagnatude,(45+180) % 360,true);
      backLeftVector = new Vector(rotationalMagnatude,(225+180) % 360,true);
      backRightVector = new Vector(rotationalMagnatude,(315+180) % 360,true);
    }
    else{
      //System.out.println("Hello");
      frontLeftVector = new Vector(rotationalMagnatude,135,true);
      frontRightVector = new Vector(rotationalMagnatude,(45),true);
      backLeftVector = new Vector(rotationalMagnatude,225,true);
      backRightVector = new Vector(rotationalMagnatude,315,true);
    }
    frontLeftVector = frontLeftVector.addVector(driveVector);
    frontRightVector = frontRightVector.addVector(driveVector);
    backLeftVector = backLeftVector.addVector(driveVector);
    backRightVector = backRightVector.addVector(driveVector);
    
    frontLeftModule.drive(frontLeftVector.getMagnatude(),frontLeftVector.getDegree(),joystickOff,lockedOn,autoMode);
    frontRightModule.drive(-frontRightVector.getMagnatude(),frontRightVector.getDegree(),joystickOff,lockedOn,autoMode);
    backLeftModule.drive(backLeftVector.getMagnatude(),backLeftVector.getDegree(),joystickOff,lockedOn,autoMode);
    backRightModule.drive(backRightVector.getMagnatude(),backRightVector.getDegree(),joystickOff,lockedOn,autoMode);
  }
  public void setLockedOn(boolean lockedOn){
    this.lockedOn = lockedOn;
    //System.out.println(lockedOn);
  }
  public boolean getLockedOn(){
    return lockedOn;
  }
  public void setLockedOnRotationalMagnatude(double rMagnatude){
    lockedOnRotationalMagnatude = rMagnatude;
  }
  public void setAutoMode(boolean autoMode){
    this.autoMode = autoMode;
  }
  public boolean getAutoMode(){
    return autoMode;
  }
  public Vector getStrafeVector(){
    return driveVector;
  }
  public void setAutoAtSetPoint(boolean atAutoAtSetPoint){
    this.autoAtSetPoint = atAutoAtSetPoint;
  }
  public boolean getAtAutoSetPoint(){
    return autoAtSetPoint;
  }
  public void setAutoPos(){
    originDX = dx;
    originDY = dy;
  }
  public void resetAutoAtSetPoint(){
    autoAtSetPoint = false;
  }
  public void autoDrive(double xfeet,double yfeet,double rotation,boolean inbetweenSetPoint){
    //autoMode = false;
    //double xMeters = xfeet/3.28084;
    //double yMeters = yfeet/3.28084;
    dx = odometer.getCenterPosition()[0];
    //System.out.println(dx);
    dy = odometer.getCenterPosition()[1];
    //System.out.println(dy);
    
    double multiplier;
    if (inbetweenSetPoint){
      multiplier = 10;
    }
    else{
      multiplier = 1;
    }
    xfeet = originDX + xfeet;
    yfeet = originDY + yfeet;
    xCalculate = xDisplacementController.calculate(dx,(xfeet) * multiplier);
    yCalculate = yDisplacementController.calculate(dy,(yfeet) * multiplier);
    /*if (!inbetweenSetPoint){
      if (((Math.abs(xfeet-dx))-Math.abs(xfeet*multiplier)<=-Math.abs(xfeet * multiplier) + 0.5) && (Math.abs(yfeet-dy)-Math.abs(yfeet*multiplier)<=-Math.abs(yfeet * multiplier) + 0.5)  &&(Math.abs(degreeOffset-rotation)<7)){
        //if (xDisplacementController.atSetpoint() && yDisplacementController.atSetpoint()){
        //drive(0,0,0);
        //System.out.println("AutoDriveFinished");
        autoAtSetPoint = true;
        stopDriving();
      }
      else{
        autoAtSetPoint = false;
      }
    }
    else{*/
      if (inbetweenSetPoint){
        if (Math.abs(xfeet - dx)<= 0.6){
          xCalculate = xDisplacementController.calculate(dx,xfeet);
        }
        if (Math.abs(yfeet - dy)<= 0.6){
          yCalculate = yDisplacementController.calculate(dy, yfeet);
        }
      }
      if ((Math.abs(xfeet-dx)<=0.7) && (Math.abs(yfeet-dy)<=0.7) &&(Math.abs(degreeOffset-rotation)<7)){
        autoAtSetPoint = true;
        stopDriving();
        System.out.println("FINISHED");
      }
      else{
        autoAtSetPoint= false;
      }
    //}
    Vector vector = new Vector(xCalculate,yCalculate);
    double rCalculate = autoDriveToDegree((getDegreeOffset()),rotation);
    if (!autoAtSetPoint){
      drive(vector.getMagnatude(),vector.getDegree(),rCalculate);
    }
    else{
      stopDriving();
      //drive(0,0,0);
    }
    
  }
  
  public double getAbsoluteRotation(){
    //return pigeon.getAbsoluteCompassHeading();//WHCH WAY DOES IT ROTATE
    return pigeon.getAngle();
  }
  public void setJoystickOff(boolean joystickOff){
    this.joystickOff = joystickOff;
  }

  public double driveToDegree(double currentDegree, double targetDegree){
    double value = 0;
    rotateToDegreeController.calculate(currentDegree,targetDegree);
    value = rotateToDegreeController.calculate(currentDegree,targetDegree);
    value = MathUtil.clamp(value,-1,1);
    return value;
  }
  public double autoDriveToDegree(double currentDegree, double targetDegree){
    double value = 0;
    autoRotateToDegreeController.calculate(currentDegree,targetDegree);
    value = autoRotateToDegreeController.calculate(currentDegree,targetDegree);
    value = MathUtil.clamp(value,-1,1);
    return value;
  }
  public double getDegreeOffset(){
    return degreeOffset;
  }
  public void toggleChangeSpeed(){
    highSpeed = !highSpeed;
    if (highSpeed){
      strafeMultiplier = 0.90;
    }
    else{
      strafeMultiplier = 0.5;
    }
  }
  public void stopDriving(){
    frontLeftDriver.set(0);
    frontLeftTurner.set(0);
    frontRightDriver.set(0);
    frontRightTurner.set(0);
    backLeftDriver.set(0);
    backLeftTurner.set(0);
    backRightTurner.set(0);
    backRightDriver.set(0);
    
  }
  public SwerveModule getFrontLeftModule(){
    return frontLeftModule;
  }
  public SwerveModule getFrontRightModule(){
    return frontRightModule;
  }
  public SwerveModule getBackLeftModule(){
    return backLeftModule;
  }
  public SwerveModule getBackRightModule(){
    return backRightModule;
  }
  public TalonFX getFrontLeftDriver(){
    return frontLeftDriver;
  }
  public TalonFX getFrontRightDriver(){
    return frontRightDriver;
  }
  public TalonFX getBackLeftDriver(){
    return backLeftDriver;
  }
  public TalonFX getBackRightDriver(){
    return backRightDriver;
  }
  public double getXDisplacement(){
    return xDisplacement;
  }
  public double getYDisplacement(){
    return yDisplacement;
  }
  public void resetOnce(){
    once = true;
  }
  @Override
  public void periodic() {
    //System.out.println(autoMode);
    //System.out.println(frontLeftTurner.get());
    if(once){
      originalDegree = pigeon.getAngle();
      originalDegree = (((originalDegree%360) + 360) % 360);
      once = false;
    }
    degreeOffset = pigeon.getAngle() - originalDegree;
    degreeOffset = ((-degreeOffset % 360) + 360) % 360;
    //System.out.println(degreeOffset);
    if (!isRotating){
      if (instance){

        originalInstance = degreeOffset;
        instance = false;
      }
      directionCorrector.calculate(degreeOffset,originalInstance);
      if (!directionCorrector.atSetpoint()){
        directionCorrectorValue = directionCorrector.calculate(degreeOffset,originalInstance);
        directionCorrectorValue = MathUtil.clamp(directionCorrectorValue,-1,1);
      }
    }
    else{
      instance = true;
    }
    if (autoMode){
      flDifferenceTick = Math.abs(frontLeftDriver.getPosition().getValueAsDouble() - flPreviousTick);
      frDifferenceTick = Math.abs(frontRightDriver.getPosition().getValueAsDouble() - frPreviousTick);
      blDifferenceTick = Math.abs(backLeftDriver.getPosition().getValueAsDouble() - blPreviousTick);
      brDifferenceTick = Math.abs(backRightDriver.getPosition().getValueAsDouble() - brPreviousTick);
      flDifferenceTick = flDifferenceTick / SwerveDriveConstants.ticksPerRotation * SwerveDriveConstants.feetPerRotation;
      frDifferenceTick = frDifferenceTick / SwerveDriveConstants.ticksPerRotation * SwerveDriveConstants.feetPerRotation;
      blDifferenceTick = blDifferenceTick / SwerveDriveConstants.ticksPerRotation * SwerveDriveConstants.feetPerRotation;
      brDifferenceTick = brDifferenceTick / SwerveDriveConstants.ticksPerRotation * SwerveDriveConstants.feetPerRotation;
      
      flPreviousTick = frontLeftDriver.getPosition().getValueAsDouble();
      frPreviousTick = frontRightDriver.getPosition().getValueAsDouble();
      blPreviousTick = backLeftDriver.getPosition().getValueAsDouble();
      brPreviousTick = backRightDriver.getPosition().getValueAsDouble();
      //System.out.println(frontLeftModule.getGeneralModuleDegree() + degreeOffset);

      Vector flv = new Vector(flDifferenceTick,(frontLeftModule.getGeneralModuleDegree()+degreeOffset) % 360,true);
      Vector flr = new Vector(frDifferenceTick,(frontRightModule.getGeneralModuleDegree()+degreeOffset) % 360,true);
      Vector blv = new Vector(blDifferenceTick,(backLeftModule.getGeneralModuleDegree()+degreeOffset)%360,true);
      Vector brv = new Vector(brDifferenceTick,(backRightModule.getGeneralModuleDegree()+degreeOffset) % 360,true);
      odometer.update(flv,flr,blv,brv);
      //System.out.println(odometer.centerX + ", " + odometer.centerY);
    }
    SmartDashboard.putNumber("DY", odometer.centerY);
    SmartDashboard.putNumber("DX",odometer.centerX);
    SmartDashboard.putNumber("Degree", degreeOffset);
    SmartDashboard.putNumber("CalculateX", xCalculate);
    SmartDashboard.putNumber("CalculateY",yCalculate);
    SmartDashboard.putBoolean("LockedON",lockedOn);
    SmartDashboard.getBoolean("Which Side",redSide);
    if (redSide){
      ampRobotDegree = 270;
    }
    else{
      ampRobotDegree = 90;
    }

  }
}
