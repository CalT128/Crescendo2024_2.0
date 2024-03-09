package frc.robot.subsystems;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class SwerveModule {
    
    TalonFX driveMotor;
    TalonFX turnMotor;
    PIDController turnController;
    
    CANcoder sensor;

    double speed;
    double invertedSpeed;
    double absolutePosition;
    double invertedAbsolutePosition;
    double targetDegree;
    double startingSensor;
    double generalModuleDegree;
    double generalModuleMagnatude;
    double previousTick;
    double currentTick;
    double differenceTick;
    boolean autoMode;
    boolean lockedOn;
    
    boolean inverted;
    public SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder sensor){
        //this.autoMode = autoMode;
        //this.lockedOn = lockedOn;
        inverted = false;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.sensor = sensor;
        //absolutePosition = 0;
        invertedAbsolutePosition = 0;
        targetDegree = 0;
        if (sensor.getDeviceID() == 24){
            startingSensor = Constants.SwerveDriveConstants.frontLeftSensor;
        }
        else if (sensor.getDeviceID() == 21){
            startingSensor = Constants.SwerveDriveConstants.frontRightSensor;
        }
        else if (sensor.getDeviceID() == 22){
            startingSensor = Constants.SwerveDriveConstants.backRightSensor;
        }
        else if (sensor.getDeviceID() == 23){
            startingSensor = Constants.SwerveDriveConstants.backLeftSensor;
        }
        turnController = new PIDController(Constants.SwerveDriveConstants.kP, Constants.SwerveDriveConstants.kI, Constants.SwerveDriveConstants.kD);
        turnController.enableContinuousInput(0, 360);
        turnController.setTolerance(0.001);
        generalModuleDegree = 0;
        generalModuleMagnatude = 0;
        previousTick = 0;
        currentTick = 0;
        differenceTick = 0;
    }
    public void drive(double speed, double targetDegree,boolean joystickOff,boolean lockedOn){
        //turnMotor.setNeutralMode(NeutralModeValue.Brake);
        //driveMotor.setNeutralMode(NeutralModeValue.Brake);
        this.speed = speed;
        this.invertedSpeed = speed * -1;

        this.targetDegree = targetDegree;
        turnController.setSetpoint(this.targetDegree);
        
        double canCoderPosition = sensor.getAbsolutePosition().getValue();
        if (canCoderPosition>0){
            canCoderPosition *= 360;
            canCoderPosition += 180;
        }
        else{
            canCoderPosition *=-360;
            canCoderPosition = 180-canCoderPosition;
        }
        //System.out.println(canCoderPosition);
        //System.out.println(sensor.getAbsolutePosition().getValue());
        this.absolutePosition = canCoderPosition-startingSensor;

        if (this.absolutePosition < 0){
            this.absolutePosition += 360;
        }
        generalModuleDegree = absolutePosition;
        
        
        absolutePosition = Math.abs((this.absolutePosition+180) % 360);
        invertedAbsolutePosition = Math.abs((this.absolutePosition+180) % 360);
        if (!inverted){
            turnController.calculate(this.absolutePosition,this.targetDegree);
            generalModuleDegree = absolutePosition;////////
            if (turnController.getPositionError()< 90){
                if (!joystickOff){
                    turnMotor.set(-turnController.calculate(this.absolutePosition,this.targetDegree));
                    driveMotor.set(this.speed);
                }
                else if (!lockedOn){
                    turnMotor.set(0);
                    driveMotor.set(0);
                }
            }
            else{
                inverted = true;
            }
        }
        else{
            turnController.calculate(this.invertedAbsolutePosition,this.targetDegree);
            generalModuleDegree = invertedAbsolutePosition;/////////
            if (turnController.getPositionError() < 90){
                if (!joystickOff){
                    turnMotor.set(-turnController.calculate(this.invertedAbsolutePosition,this.targetDegree));
                    driveMotor.set(-this.speed);
                }
                else if (!lockedOn){
                    turnMotor.set(0);
                    driveMotor.set(0);
                }
            }
            else{
                inverted = false;
            }
        }
        generalModuleMagnatude = Math.abs(speed);
        if (turnController.atSetpoint()){
            turnMotor.stopMotor();
        }
    }
    
    public void stop(){
        turnMotor.stopMotor();
        driveMotor.stopMotor();
    }
    public double getCalculate(){
        return turnController.calculate(absolutePosition,targetDegree);
    }
    public double getAbsolutePosition(){
        return absolutePosition;
    }
    public boolean getAtSetPoint(){
        return turnController.atSetpoint();
    }
    public double getInvertedAbsolutePosition(){
        return invertedAbsolutePosition;
    }
    public double getTargetDegree(){
        return targetDegree;
    }
    public double getSpeed(){
        return speed;
    }
    public double getGeneralModuleDegree(){
        return generalModuleDegree;
    }
    public double getGeneralModuleMagnatude(){
        return generalModuleMagnatude;
    }
    /*public double getDriverMotorTicks(){
        return
    }*/
}
