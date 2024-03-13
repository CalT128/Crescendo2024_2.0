// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
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
  PhotonTrackedTarget value;
  PhotonPipelineResult frame;

  double xOffset;
  double yOffset;

  boolean hasTarget;
  boolean isAligned;

  double targetDistance;

  SwerveSubsystem m_swerve;
  ShooterSubsystem m_shooter;

  public PhotonVisionSubsystem(SwerveSubsystem swerve, ShooterSubsystem shooter) {
    camera = new PhotonCamera("photonvision");

    isAligned = false;

    m_swerve = swerve;
    m_shooter = shooter;
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
    if (hasTarget){
      if(((xOffset < -VisionConstants.X_ALIGNMENT_RANGE) || (xOffset > VisionConstants.X_ALIGNMENT_RANGE)) && !isAligned){
        m_swerve.setLockedOn(true);
        if(xOffset < 0){
          xOffset += 360;
        }
        double rotationalMagnitude = m_swerve.driveToDegree(xOffset, 0);
        m_swerve.setLockedOnRotationalMagnatude(rotationalMagnitude);
      }
    }
    else{
      m_swerve.setLockedOn(false);
    }
  }

  public void correctLauncher(){
    if(hasTarget){
      double angleCalculated = (90 - Math.toDegrees((Math.atan((VisionConstants.SPEAKER_HEIGHT - 
        VisionConstants.LIME_HEIGHT) / distanceToTarget(yOffset)))) - VisionConstants.INITIAL_SHOOTER_ANGLE);
      double angleWanted = VisionConstants.DEGREES_TO_ROTATIONS * angleCalculated;
        if (angleWanted < 0){
          angleWanted = 0;
        }
        m_shooter.setSpeakerPosition(angleWanted);
    }
    else{
      m_shooter.setSpeakerPosition(Constants.ShooterConstants.shooterSpeakerPosition);
    }
  }

  @Override
  public void periodic() {
    hasTarget = frame.hasTargets();
    value = frame.getBestTarget();
    xOffset = value.getYaw();
    yOffset = value.getPitch();

    if(Math.abs(xOffset) >= VisionConstants.X_ALIGNMENT_RANGE)
      isAligned = false;
    else
      isAligned = true;

    SmartDashboard.putNumber("X-Offset", xOffset);
    SmartDashboard.putNumber("Y-Offset", yOffset);
    SmartDashboard.putBoolean("Has-Target", hasTarget);
    SmartDashboard.putBoolean("Is-Aligned", isAligned);

    SmartDashboard.putNumber("DistanceToTarget", targetDistance);
  }
}