// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Arrays;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPosition;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  PhotonCamera camera;
  //PhotonTrackedTarget value;
  PhotonPipelineResult frame;

  double xOffset;
  double yOffset;
  double rotationalMagnitude;

  boolean hasTarget;
  boolean isAligned;
  boolean autoAlign;

  double targetDistance;

  SwerveSubsystem m_swerve;
  ShooterSubsystem m_shooter;
  boolean isXAligned;
  boolean isYAligned;
  

  public PhotonVisionSubsystem(SwerveSubsystem swerve, ShooterSubsystem shooter) {
    camera = new PhotonCamera("photonVisionCamera");
    autoAlign = false;
    isAligned = false;
    rotationalMagnitude = 0;

    m_swerve = swerve;
    m_shooter = shooter;
    
  }
  public void setLockedOn(boolean lockedOn){
    m_swerve.setLockedOn(lockedOn);
  }

  public double distanceToTarget(double degrees){
    if(hasTarget){
      double distance = (VisionConstants.APRILTAG_HEIGHT - VisionConstants.LIME_HEIGHT) / Math.tan(Math.toRadians(degrees + VisionConstants.INITIAL_LL_ANGLE));
      return distance;
    } else {
      return 0;
    }
  }

  public void ameliorateX(){
    frame = camera.getLatestResult();
    // if we have a target and it's 4 or 7
    if(frame.hasTargets() && frame.getTargets().stream().anyMatch(t -> Arrays.asList(4,7).contains(t.getFiducialId()))){
      // get all targets
      frame.getTargets().stream()
        // filter out all except 4 or 7
        .filter(t -> Arrays.asList(4,7).contains(t.getFiducialId()))
        // set offsets of 4 or 7
        .forEach(target -> {
          xOffset = target.getYaw();
          yOffset = target.getPitch();
          hasTarget = true;
        });

      if(Math.abs(xOffset) >= VisionConstants.X_ALIGNMENT_RANGE)
        isAligned = false;
      else
        isAligned = true;
      
      if(!isAligned){
        //System.out.println("auto algn");
        m_swerve.setLockedOn(true);

        //if(((xOffset < -VisionConstants.X_ALIGNMENT_RANGE) || (xOffset > VisionConstants.X_ALIGNMENT_RANGE)) && !isAligned){
        rotationalMagnitude = m_swerve.driveToDegree(xOffset, 2.5);
        rotationalMagnitude *=2.4;
        m_swerve.setLockedOnRotationalMagnatude(rotationalMagnitude);
        if(m_swerve.getAutoMode()){
          m_swerve.drive(0,0,rotationalMagnitude);
        }
        //m_swerve.drive(0,0,rotationalMagnitude);
        if (Math.abs(xOffset)<7){
          isXAligned = true;
        }
        else{
          isXAligned = false;
        }
        //System.out.println(rotationalMagnitude);
        //}
      }
    }
    else{
      hasTarget = false;
      m_swerve.setLockedOn(false);
      isXAligned = false;
    }
  }
  public void resetIsAligned(){
    isYAligned = false;
    isXAligned = false;
  }
  public boolean getIsXAligned(){
    return isXAligned;
  }
  public boolean getIsYAligned(){
    return isYAligned;
  }
  public void correctLauncher(){
    frame = camera.getLatestResult();
    if(frame.hasTargets() && frame.getTargets().stream().anyMatch(t -> Arrays.asList(4,7).contains(t.getFiducialId()))){
      frame.getTargets().stream()
        .filter(t -> Arrays.asList(4,7).contains(t.getFiducialId()))
        .forEach(target -> {
          xOffset = target.getYaw();
          yOffset = target.getPitch();
          hasTarget = true;
        });
      // for (PhotonTrackedTarget target:targets){
      //   SmartDashboard.putNumber("target number",target.getFiducialId());
      //   if (target.getFiducialId() == 4 || target.getFiducialId() == 7){
      //     xOffset = target.getYaw();
      //     yOffset = target.getPitch();
      //     hasTarget = true;
      //   }
      // }
      double angleCalculated = (90 - Math.toDegrees((Math.atan((VisionConstants.SPEAKER_HEIGHT - 
        VisionConstants.LIME_HEIGHT) / distanceToTarget(yOffset)))) - VisionConstants.INITIAL_SHOOTER_ANGLE);
      double angleWanted = VisionConstants.DEGREES_TO_ROTATIONS * angleCalculated;
      if (angleWanted < 0){
        angleWanted = 0;
      }
      m_shooter.setSpeakerPosition(angleWanted);
      if (Math.abs(angleWanted - m_shooter.getCurrentRotations()) < 0.06){
        isYAligned = true;
      }
      else{
        isYAligned = false;
      }

    }
    else{
      isYAligned = false;
      hasTarget = false;
      m_shooter.setSpeakerPosition(Constants.ShooterConstants.shooterSpeakerPosition);
    }
  }
  public void setAutoAlign(boolean autoAlign){
    this.autoAlign = autoAlign;
  }
  @Override
  public void periodic() {
    //System.out.println(hasTarget);
    //System.out.println(camera.isConnected());
    SmartDashboard.putNumber("VISION X OFFSET",xOffset);
    SmartDashboard.putBoolean("ISYALIGNED:", isYAligned);
    SmartDashboard.putBoolean("ISXALIGNED",isXAligned);
    
    //value = frame.getBestTarget();
    // var targets = frame.getTargets();
    // for (PhotonTrackedTarget target:targets){
    //   if (target.getFiducialId() == 4 || target.getFiducialId() == 7){
    //     xOffset = target.getYaw();
    //     yOffset = target.getPitch();
    //   }
    //   else{
    //     xOffset = 0;
    //     yOffset = 0;
    //   }
    // }
    if(!hasTarget){
      xOffset = 0;
      yOffset = 0;
    }

    
    if (autoAlign){
      //System.out.println("HELLO");
      m_shooter.setShooterPosition(ShooterPosition.SPEAKER);
      ameliorateX();
      correctLauncher();
    }

    // SmartDashboard.putNumber("X-Offset", xOffset);
    // SmartDashboard.putNumber("Y-Offset", yOffset);
    // SmartDashboard.putNumber("rotationalMagnitude",rotationalMagnitude);
    SmartDashboard.putBoolean("Has-Target", hasTarget);
    SmartDashboard.putBoolean("Is-Aligned", isAligned);
    SmartDashboard.putNumber("DistanceToTarget", targetDistance);
  }
}