package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class SwerveOdometer {
    Vector frontLeftModule;
    Vector frontRightModule;
    Vector backLeftModule;
    Vector backRightModule;
    Vector flcVector;
    Vector frcVector;
    Vector combinedVectors;
    double halfSideLength;
    double centerX;
    double centerY;
    double[] centerPosition;

    public SwerveOdometer(){
        halfSideLength = Constants.SwerveDriveConstants.robotSideLength/2;//CHANGE
        frontLeftModule = new Vector(-halfSideLength,halfSideLength);
        frontRightModule = new Vector(halfSideLength,halfSideLength);
        backLeftModule = new Vector(-halfSideLength,-halfSideLength);
        backRightModule = new Vector(halfSideLength,-halfSideLength);
    }
    public void update(Vector flM,Vector frM, Vector blM, Vector brM){
        frontLeftModule.addVectorX(flM.getVectorX());
        frontLeftModule.addVectorY(flM.getVectorY());
        //System.out.println(flM.getVectorY());
        
        frontRightModule.addVectorX(frM.getVectorX());
        frontRightModule.addVectorY(frM.getVectorY());
        
        backLeftModule.addVectorX(blM.getVectorX());
        backLeftModule.addVectorY(blM.getVectorY());
        
        backRightModule.addVectorX(brM.getVectorX());
        backRightModule.addVectorY(brM.getVectorY());
        
        centerX = (frontLeftModule.getVectorX()+backRightModule.getVectorX())/2;
        centerY = (frontLeftModule.getVectorY()+backRightModule.getVectorY())/2;
        double[] array = {centerX,centerY};
        centerPosition = array;
        centerPosition = getCenterPosition();
    }
    public double[] getCenterPosition(){
        return centerPosition;
    }
    public double getAngle(){
        flcVector = new Vector(frontLeftModule.getVectorX()-centerPosition[0],frontLeftModule.getVectorY()-centerPosition[1]);
        frcVector = new Vector(frontRightModule.getVectorX()-centerPosition[0],frontRightModule.getVectorY()-centerPosition[1]);
        combinedVectors = flcVector.addVector(frcVector);
        return combinedVectors.getDegree();
    }
}
