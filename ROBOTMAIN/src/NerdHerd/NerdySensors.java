/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package NerdHerd;

import NerdHerd.Source.NerdyBotThreaded;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author student
 */
public class NerdySensors extends Thread{
    private NerdyAccel accel;
    private Gyro gyro;
    private double heading = 0.0, gyroRate = 0.0;
    private double  xAxis, yAxis, zAxis, xAxisOffset = 0, yAxisOffset = 0;
    private TrapezoidalIntegrator distanceIntegrator, speedIntegrator;
    private double distanceTraveled = 0.0, speedTraveling = 0.0, acceleration = 0.0;
    public boolean sensorActive = true, dataReportingActive = true;
    public NerdySensorData sensorData;
    
    public NerdySensors(){
        gyro = new Gyro(2);
        accel = new NerdyAccel(1, NerdyAccel.DataFormat_Range.k4G);
        distanceIntegrator =  new TrapezoidalIntegrator(NerdyBotThreaded.k_PeriodTime,50);
        speedIntegrator =  new TrapezoidalIntegrator(NerdyBotThreaded.k_PeriodTime,50);
        calibrate();
        
        sensorData = new NerdySensorData();
        sensorData.setPriority(1);
        sensorData.start();        
    }
    
    private void calibrate(){
        //Calibrate Gyro
        gyro.setSensitivity(.125);
        //Calibrate Accel
        int calibrationStepsMax = 100;
        double xAxisSum = 0;
        double yAxisSum = 0;
        for(int i = 0;i<calibrationStepsMax;i++){
            xAxisSum += accel.getAcceleration(NerdyAccel.Axes.kX);
            yAxisSum += accel.getAcceleration(NerdyAccel.Axes.kY);
            Timer.delay(NerdyBotThreaded.k_PeriodTime);            
        }
        xAxisSum /= calibrationStepsMax;
        yAxisSum /= calibrationStepsMax;
        xAxisOffset = xAxisSum;
        yAxisOffset = yAxisSum;
        System.out.println(xAxisSum);
        System.out.println(yAxisSum);
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
        gyroRate = gyro.getRate();
    }
    
    private void updateAccelValues(){
        xAxis = accel.getAcceleration(NerdyAccel.Axes.kX) - xAxisOffset;
        yAxis = accel.getAcceleration(NerdyAccel.Axes.kY) - yAxisOffset;
        zAxis = accel.getAcceleration(NerdyAccel.Axes.kZ);
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
        while(sensorActive){
            double startTime = getSeconds();
            System.out.println("Hello. I am Nerdy Sensor.");
            updateGyroValues();
            updateAccelValues();
            
            double timePassed = getSeconds()-startTime;
            double sleepTime = NerdyBotThreaded.k_PeriodTime-(timePassed);//seconds
            SmartDashboard.putDouble("TimePassedSensor", timePassed);
            SmartDashboard.putDouble("SleepTimeSensor", sleepTime);
            if(sleepTime < 0){
                sleepTime = 0;
            }
            try{
                sleep((long)(sleepTime*1000));//milliseconds
            }catch(Exception e){
                System.out.println(e);
                //sensorActive = false;
            }
        }   
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
    
    public class NerdySensorData extends Thread{
        public NerdySensorData(){

        }
        
        public void run(){
            while(dataReportingActive){    
                System.out.println("Hello. I am Nerdy Sensor Data.");
                SmartDashboard.putDouble("xAxis" , xAxis);
                SmartDashboard.putDouble("yAxis" , yAxis);
                SmartDashboard.putDouble("zAxis" , zAxis);
                SmartDashboard.putDouble("acceleration" , acceleration);
                SmartDashboard.putDouble("speedTraveling" , speedTraveling);
                SmartDashboard.putDouble("distanceTraveled" , distanceTraveled*32);
            }
        }
        
    }
    
}
