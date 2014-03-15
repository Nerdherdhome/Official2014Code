/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package NerdHerd;
import NerdHerd.NerdyAccel;
import NerdHerd.Source.NerdyBot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Gyro;
import NerdHerd.TrapezoidalIntegrator;
import NerdHerd.LowPassFilter;
import NerdHerd.NerdyAccel;

/**
 *
 * @author student
 */
public class NerdySensors {
    NerdyAccel accel;
    Gyro gyro;
    double heading = 0.0;
    double gyroRate = 0.0;
    double distanceTraveled = 0.0;
    private double xAxisOffset = 0, yAxisOffset = 0;
    TrapezoidalIntegrator DistanceIntegrator, speedIntegrator;
    double speedTraveling = 0.0;
    private double headingTolerance;
    
    public NerdySensors(){
        
        DistanceIntegrator = new TrapezoidalIntegrator(NerdyBot.k_PeriodTime,16);
        speedIntegrator = new TrapezoidalIntegrator(NerdyBot.k_PeriodTime, 4.5);
        accel = new NerdyAccel(1, NerdyAccel.DataFormat_Range.k4G);
        gyro = new Gyro(2);
        init();
    }
    
    public void init(){
        
        int calibrationStepsMax = 100;
        double xAxisSum = 0;
        double yAxisSum = 0;
        for(int i = 0;i<calibrationStepsMax;i++){
            xAxisSum += accel.getAcceleration(NerdyAccel.Axes.kX);
            yAxisSum += accel.getAcceleration(NerdyAccel.Axes.kY);
            Timer.delay(NerdyBot.k_PeriodTime);            
        }
        xAxisSum /= calibrationStepsMax;
        yAxisSum /= calibrationStepsMax;
        xAxisOffset = xAxisSum;
        yAxisOffset = yAxisSum;
        System.out.println(xAxisSum);
        System.out.println(yAxisSum);
    }
    
    private void updateGyroValues(){
        heading = (-gyro.getAngle()+720) % 360;
        gyroRate = gyro.getRate();
    }
    
    public double getRate(){
        
        return gyroRate;
    }
    
    public void reset(){
        gyro.reset();
        DistanceIntegrator.resetAccumulation();
        speedIntegrator.resetAccumulation();
        speedTraveling = 0;
        init();
    }

    public double getHeading(){
    
    /*
    Returns the last updated Heading.
    */
    
        return heading;
    }
    
    public double calcDistanceTraveled(){
        
    /*
    Calculates the distance traveled
    */
        
        
        double xAxis = accel.getAcceleration(NerdyAccel.Axes.kX) - xAxisOffset;
        double yAxis = (accel.getAcceleration(NerdyAccel.Axes.kY) - yAxisOffset);
        double zAxis = accel.getAcceleration(NerdyAccel.Axes.kZ);
        double acceleration = (yAxis);
        speedTraveling = speedIntegrator.updateAccumulation(acceleration);
        distanceTraveled = DistanceIntegrator.updateAccumulation(speedTraveling);
        SmartDashboard.putDouble("xAxis" , xAxis);
        SmartDashboard.putDouble("yAxis" , yAxis);
        SmartDashboard.putDouble("zAxis" , zAxis);
        SmartDashboard.putDouble("acceleration" , acceleration);
        SmartDashboard.putDouble("speedTraveling" , speedTraveling);
        SmartDashboard.putDouble("distanceTraveled" , distanceTraveled*32);
        
        return distanceTraveled;
    }
    
    public double getDistanceTraveled(){
    
    /*
    Returns the last updated distance traveled.
    Must use calcDistanceTraveled() prior to get the most recent result.
    */
    
        return distanceTraveled;
    }
    
    public void setHeadingTolerance(double degree){
    
    /*
    This sets a tolerance for the heading in degrees.
    Heading is tolerable if is + or - tolerance.
    Default is 0.
    */
    
        headingTolerance = degree;
    }
    
    public double getHeadingTolerance(){
    
    /*
    Returns heading tolerance in degrees. 
    If not set, this should return 0.
    */
    
        return headingTolerance;
    }
    
    public void resetDistance(){
        distanceTraveled = 0;
    }
    
    
   
    

}
