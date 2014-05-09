/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package NerdHerd;

import NerdHerd.Source.NerdyBot;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class NerdyPIDController{

    private double headingTolerance, distanceTolerance;
    private double error;
    private double heading;
    private double distanceTraveled = 0;
    private double KpAngular = 0.33333;
    private double KiAngular = 0.1;
    private double KdAngular = 1;
    private double angularScaleFactor = 0.03333333;
    private double KpLinear = 1/3;
    private double KiLinear = 1/.45;
    private double KdLinear = 0.5;
    private double distanceRemaining = 0;
    public CANJaguar LtDriveMain, LtDriveSub1, LtDriveSub2, RtDriveMain, RtDriveSub1, RtDriveSub2;
    public TrapezoidalIntegrator DistanceRemainingIntegrator;
    public NerdySensors Sensor;
            
    public NerdyPIDController(){
        try{
            LtDriveMain = new CANJaguar(3);
            LtDriveSub1 = new CANJaguar(5);
            LtDriveSub2 = new CANJaguar(7);
            RtDriveMain = new CANJaguar(2);
            RtDriveSub1 = new CANJaguar(4);
            RtDriveSub2 = new CANJaguar(6);
        }catch(Exception e){
            System.out.println(e);
        }
        DistanceRemainingIntegrator = new TrapezoidalIntegrator(NerdyBot.k_PeriodTime);
        Sensor = new NerdySensors();
    }
    
/********************************
Setter / Getter / Logic Functions
*******************************/
    
    
    public void setHeadingTolerance(double degree){
        headingTolerance = degree;
    }
 
    public boolean isHeadingTolerable(double desiredAngle){
        heading = Sensor.getHeading();
        return ((desiredAngle < heading+headingTolerance) && (desiredAngle > heading-headingTolerance));
    }
    
    
    public void setDistanceTolerance(double feet){
    
        distanceTolerance = feet;
    }
    
    public boolean isDistanceTolerable(double desiredDistance){
        double distance_Traveled = Sensor.getDistanceTraveled();
        return ((desiredDistance < distance_Traveled+distanceTolerance) && (desiredDistance > distance_Traveled-distanceTolerance));
    }

    
    public double calcDistanceRemaining(double desiredDistance){
        
        distanceTraveled = Sensor.getDistanceTraveled();
        distanceRemaining = desiredDistance - Math.abs(distanceTraveled);
        return distanceRemaining;
    }
    
    private double calcShortestRotation(double desiredAngle){
        heading = Sensor.getHeading();
        error = desiredAngle - heading;
        if (Math.abs(error) >  180){
            error = -sign(error)*(360 - Math.abs(error));
        }
        return error;
    }   
    
/********************************
PID Functions
*******************************/
    
    public double getPIDOutputAngular(double desiredAngle){

        double err = calcShortestRotation(desiredAngle);
        //double rateCommand = (err * KpAngular) + (HeadingIntegrator.updateAccumulation(err) * KiAngular);
        //double PIDOutputAngular = ((rateCommand - gyroRate) * KdAngular*angularScaleFactor);
        double PIDOutputAngular = ((err * KpAngular) * KdAngular*angularScaleFactor);
   
        PIDOutputAngular = constrain(PIDOutputAngular,1);
        //angularScaleFactor is to normalize the output to -1 - 1
        
        SmartDashboard.putDouble("Angular Error", err);
        return PIDOutputAngular;
    }
    
    public double getPIDOutputLinear(double desiredDistance){

        calcDistanceRemaining(desiredDistance);
        double P = distanceRemaining * KpLinear;
        double I = DistanceRemainingIntegrator.updateAccumulation(distanceRemaining) * KiLinear;
        double PIDOutputLinear = (P + I) * KdLinear; 
        PIDOutputLinear = constrain(PIDOutputLinear, 1);
        //double PIDOutputLinear = P;
        return PIDOutputLinear;
    }
    
/********************************
Movement Functions
*******************************/
    
    public void moveAndRotate(double desiredAngle, double linearPower){

        double ltPower, rtPower;
        double angularPower = getPIDOutputAngular(desiredAngle);
        double scaleFactor = 1-Math.abs(angularPower);//scaleFactor is to favor the rotation 
        ltPower = angularPower*0.5 + linearPower*0.5;
        rtPower = angularPower*0.5 - linearPower*0.5;
        
        ltPower = constrain(ltPower, 1);
        rtPower = constrain(rtPower, 1);
        
        try{
            LtDriveMain.set(ltPower);
            LtDriveSub1.set(ltPower);
            LtDriveSub2.set(ltPower);
            RtDriveMain.set(rtPower);
            RtDriveSub1.set(rtPower);
            RtDriveSub2.set(rtPower);
        }catch (Exception e){
            System.out.println(e);
        }
    }
    
    public void rotate(double angularPower){
        try{
            LtDriveMain.set(angularPower);
            LtDriveSub1.set(angularPower);
            LtDriveSub2.set(angularPower);
            RtDriveMain.set(angularPower);
            RtDriveSub1.set(angularPower);
            RtDriveSub2.set(angularPower);
        }catch (Exception e){
            System.out.println(e);
        }
    }
    
    public void move(double linearPower){
        try{
            LtDriveMain.set(linearPower);
            LtDriveSub1.set(linearPower);
            LtDriveSub2.set(linearPower);
            RtDriveMain.set(-linearPower);
            RtDriveSub1.set(-linearPower);
            RtDriveSub2.set(-linearPower);
        }catch (Exception e){
            System.out.println(e);
        }
    }
    
    public void stopMotor(){
        String JoystickStatus = SmartDashboard.getString("Joystick Alert Message");
    }
        
/********************************
Helper Functions
*******************************/
    
    
    public int sign(double number){
        if (number > 0){
            return 1;
        }else if (number < 0){
            return -1;
        } else {
            return 0;
        }
    }
    
    private double constrain (double value, double m_limit){
        if(value > m_limit){
            value = m_limit;
        }else if (value < -m_limit){
            value = -m_limit;
        }
        return value; 
    }
    
    public void reset(){
        Sensor.reset();
    }
} 
