// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveDriveConstants {
    public static final double kP = 0.0037;
    public static final double kI = 0.00001;
    public static final double kD = 0.0;
    public static final double rKP = 0.0050;
    public static final double rKI = 0.002;
    public static final double rKD = 0.001;
    public static final double frontLeftSensor = 272.4;
    public static final double frontRightSensor = 159.5;
    public static final double backLeftSensor = 16.4;
    public static final double backRightSensor = 283.899;
    public static final double robotSideLength = 29.5;//INCH
    public static final double robotWheelDiameter = 4;//INCH
    public static final double ticksPerRotation = 8.1552734;
    public static final double feetPerRotation = (4 * Math.PI)/12;
  }
  public static class ShooterConstants{
    public static final double shooterAngleConstants = 103.216632;
    public static final double shooterIntakePosition = 0.615;
    //public static final double shooterAmpPosition = 0.124886;
    public static final double shooterAmpPosition = 0.13;
    public static final double shooterSpeakerPosition = 0.28;
    public static final double shooterClimbPosition = 2.25;
    public static final double shooterDefaultPosition = 0.00;
    public static final double kP = 5.3;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
  }
  public static enum ShooterPosition{
    INTAKE,
    AMP,
    SPEAKER,
    CLIMB,
    
    DEFAULT
  }
  public static class VisionConstants {
    public static final double INITIAL_LL_ANGLE = 24.71; //degrees
    public static final double SPEAKER_HEIGHT = 6; //feet 
    //public static final double LIMELIGHT_HEIGHT = 0; //height to limelight in feet MEASURE
    //TEST VARIABLES
    public static final double APRILTAG_HEIGHT = 4.041666667;
    public static final double LIME_HEIGHT = 1.4583333333;
    public static final double INITIAL_SHOOTER_ANGLE = 10; //degrees
    public static final double DEGREES_TO_ROTATIONS = 0.011111111;


    //DISTANCE TO TARGET CONSTANTS
    public static final double DISTANCE_KP = 0;
    public static final double DISTANCE_KI = 0;
    public static final double DISTANCE_KD = 0;

    public static final double OPTIMAL_DISTANCE_OF_ROBOT = 0; //feet

    public static final double DRIVE_STRAIGHT_DEGREE = 90; //degrees

    //X ALIGNMENT TO TARGET CONSTANTS
    public static final double X_ALIGNMENT_RANGE = 0.5; //degrees
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
