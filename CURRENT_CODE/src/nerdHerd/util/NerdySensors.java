/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package NerdHerd;

import NerdHerd.Source.NerdyBot;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author student
 */
public class NerdySensors{
//    private NerdyAccel accel;
    private NerdyGyro gyro;
    private double heading = 0.0, gyroRate = 0.0;
    private double  xAxis, yAxis, zAxis, xAxisOffset = 0, yAxisOffset = 0;
    private TrapezoidalIntegrator distanceIntegrator, speedIntegrator;
    private double distanceTraveled = 0.0, speedTraveling = 0.0, acceleration = 0.0;
    public boolean sensorActive = true, dataReportingActive = true;
    
    public NerdySensors(){
        gyro = new NerdyGyro(2);//Change to 1 for expert mode!
//        accel = new NerdyAccel(1, NerdyAccel.DataFormat_Range.k4G);
        distanceIntegrator =  new TrapezoidalIntegrator(NerdyBot.k_PeriodTime,100);
        speedIntegrator =  new TrapezoidalIntegrator(NerdyBot.k_PeriodTime,2048);
        calibrate();
           
    }
    
    private void calibrate(){
        //Calibrate Gyro
        gyro.setSensitivity(.007);//0.0135
        //Calibrate Accel
        int calibrationStepsMax = 100;
        double xAxisSum = 0;
        double yAxisSum = 0;
//        for(int i = 0;i<calibrationStepsMax;i++){
//            xAxisSum += accel.getAcceleration(NerdyAccel.Axes.kX);
//            yAxisSum += accel.getAcceleration(NerdyAccel.Axes.kY);
//            Timer.delay(NerdyBotThreaded.k_PeriodTime);            
//        }
        xAxisSum /= calibrationStepsMax;
        yAxisSum /= calibrationStepsMax;
        xAxisOffset = xAxisSum;
        yAxisOffset = yAxisSum;
    }

    public void reset(){
        gyro.reset();
        distanceIntegrator.resetAccumulation();
        speedIntegrator.resetAccumulation();
        speedTraveling = 0;
        calibrate();
    }
    
    private void updateGyroValues(){
        heading = (gyro.getAngle()+720) % 360;
    }
    
    private void updateAccelValues(){
//        xAxis = accel.getAcceleration(NerdyAccel.Axes.kX) - xAxisOffset;
//        yAxis = accel.getAcceleration(NerdyAccel.Axes.kY) - yAxisOffset;
//        zAxis = accel.getAcceleration(NerdyAccel.Axes.kZ);
        acceleration = (yAxis);
        speedTraveling = speedIntegrator.updateAccumulation(acceleration);
        //Try Delta Y, if fail, default to updateAccumulation
        distanceTraveled = distanceIntegrator.updateAccumulationDeltaY(speedTraveling);
    }
    
    public double getDistanceTraveled(){
        return distanceTraveled;
    }
    
    public double getRate(){
        return gyroRate;
    }

    public double getHeading(){
        return heading;
    }
    
    public void run(){
        double startTime = getSeconds();
        System.out.println("Hello. I am Nerdy Sensor.");
        updateGyroValues();
        updateAccelValues();

        updateStatus();

    }
    
    /**
     * Return the system clock time in seconds. Return the time from the
     * FPGA hardware clock in seconds since the FPGA started.
     *
     * @return Robot running time in seconds.
     */
    private double getSeconds(){
        return  Utility.getFPGATime() / 1000000.0;
    }   
    

        
    public void updateStatus(){

            System.out.println("Hello. I am Nerdy Sensor Data.");
            SmartDashboard.putDouble("xAxis" , xAxis);
            SmartDashboard.putDouble("yAxis" , yAxis);
            SmartDashboard.putDouble("zAxis" , zAxis);
            SmartDashboard.putDouble("acceleration" , acceleration);
            SmartDashboard.putDouble("speedTraveling" , speedTraveling);
            SmartDashboard.putDouble("distanceTraveled" , distanceTraveled*32);
            SmartDashboard.putDouble("Gyro Angle", heading);

    }
        
    
    
}
