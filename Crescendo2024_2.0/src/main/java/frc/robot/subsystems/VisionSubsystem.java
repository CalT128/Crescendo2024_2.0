// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPosition;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry tv;
  
  double xOffset;
  double yOffset;

  boolean hasTarget;
  boolean isAligned;

  double targetDistance;

  //PIDController xAlignmentCorrector;
  // PIDController yAlignmentCorrector;
  PIDController distanceCorrector;

  SwerveSubsystem m_swerve;
  ShooterSubsystem m_shooter;

  public VisionSubsystem(SwerveSubsystem swerve, ShooterSubsystem shooter) {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    distanceCorrector = new PIDController(VisionConstants.DISTANCE_KP, VisionConstants.DISTANCE_KI, VisionConstants.DISTANCE_KD);
    //xAlignmentCorrector = new PIDController(VisionConstants.X_KP, VisionConstants.X_KI, VisionConstants.X_KD);
    // yAlignmentCorrector = new PIDController(VisionConstants.Y_KP, VisionConstants.Y_KI, VisionConstants.Y_KD);

    table.getEntry("pipeline").setValue(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

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

  // public void correctDistance(){ hahah dont need at all :))
  //   if(isAligned){
  //     if((targetDistance < VisionConstants.OPTIMAL_DISTANCE_OF_ROBOT) || (targetDistance > VisionConstants.OPTIMAL_DISTANCE_OF_ROBOT)){
  //       double direction = (targetDistance < VisionConstants.OPTIMAL_DISTANCE_OF_ROBOT ? 1 : -1);
  //       double degreeOffset = m_swerve.getDegreeOffset() * -1;
  //       Vector v = new Vector(0, (-1 * direction));
  //       if (degreeOffset < 0){
  //         degreeOffset += 360;
  //       }
  //       double rotationMagnitude = m_swerve.driveToDegree(degreeOffset, v.getDegree());
  //       double strafeMagnitude = distanceCorrector.calculate(targetDistance, VisionConstants.OPTIMAL_DISTANCE_OF_ROBOT);
  //       m_swerve.drive(strafeMagnitude, VisionConstants.DRIVE_STRAIGHT_DEGREE, rotationMagnitude);
  //     }
  //   }
  // }

  public void ameliorateX(){
    if (hasTarget){
      if(((xOffset < -VisionConstants.X_ALIGNMENT_RANGE) || (xOffset > VisionConstants.X_ALIGNMENT_RANGE)) && !isAligned){
        m_swerve.setLockedOn(true);
        if(xOffset < 0){
          xOffset += 360;
        }
        double rotationalMagnitude = m_swerve.driveToDegree(xOffset, 0);
        //System.out.println(rotationalMagnitude);
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
      // System.out.println(angleWanted);
      //double angleOffset = (angleWanted - (/*VisionConstants.DEGREES_TO_ROTATIONS**/m_shooter.getCurrentAngle()));
      //if(Math.abs(angleOffset) > 0.008){
        if (angleWanted < 0){
          angleWanted = 0;
        }
        m_shooter.setSpeakerPosition(angleWanted);
        //System.out.println("Correcting Launcher");
      /* } else {
        m_shooter.angleMotor.set(0);
      }*/
    }
    else{
      m_shooter.setSpeakerPosition(Constants.ShooterConstants.shooterSpeakerPosition);
    }
  }

  @Override
  public void periodic() {
    //System.out.println(m_swerve.getIntakeMode());
    //System.out.println(m_swerve.getIntakeMode());
    xOffset = tx.getDouble(0.0);
    yOffset = ty.getDouble(0.0);
    hasTarget = (tv.getDouble(0.0) == 1);
    targetDistance = distanceToTarget(yOffset); 

    if(Math.abs(xOffset) >= VisionConstants.X_ALIGNMENT_RANGE)
      isAligned = false;
    else
      isAligned = true;
    // CHANGE TO WHATEVER TRACKS THE DEGREES IN THE ARM IF THE LIMELIGHT IS ON THE ARM
    // OR MAYBE HAVE TO CREATE A VARIABLE FOR THE INITIAL ANGLE OF THE LIMELIGHT

    SmartDashboard.putNumber("X-Offset", xOffset);
    SmartDashboard.putNumber("Y-Offset", yOffset);
    SmartDashboard.putBoolean("Has-Target", hasTarget);
    SmartDashboard.putBoolean("Is-Aligned", isAligned);

    SmartDashboard.putNumber("DistanceToTarget", targetDistance);
  }
}