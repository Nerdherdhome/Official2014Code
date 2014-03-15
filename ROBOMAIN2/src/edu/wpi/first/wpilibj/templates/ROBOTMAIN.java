/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import NerdHerd.NerdyPIDRobot;
import NerdHerd.Source.NerdyBot;
import NerdHerd.nerdyCamera;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class ROBOTMAIN extends NerdyBot {
    /**
public class RobotTemplate extends IterativeRobot
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    
    NerdyPIDRobot Robot;
    nerdyCamera camera;
  
    
    public void robotInit() {
        Robot = new NerdyPIDRobot();
        Robot.setHeadingTolerance(3);//This is 3 degrees because it drifts 3 degrees / 2.5 minutes
        /*
        camera = new nerdyCamera(AxisCamera.ResolutionT.k320x240);
        camera.setPriority(ROBOT_TASK_PRIORITY-1);
        camera.start();
        */
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

        Robot.calcDistanceTraveled();
        updateStatus();
        
        boolean resetMode = Robot.JoystickMain.getRawButton(6);
        if(resetMode){
            reset();
        }
        
        /*
        Robot.move(Robot.getPIDOutputLinear(3));
        Robot.setHeadingTolerance(3);
        Robot.setDistanceTolerance(3);
        if (Robot.isDistanceTolerable(Robot.getDistanceTraveled())){
            Robot.move(0);
        }
        */
 
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {

        double angle = Robot.get360JoystickAngle();
        double angularPower = Robot.getPIDOutputAngular(angle);
        double linearPower =  (Robot.JoystickMain.getMagnitude()/Math.sqrt(2) ) * -Robot.sign(Robot.JoystickMain.getY());
        double twist = Robot.JoystickMain.getTwist();
        Robot.calcDistanceTraveled();
        
        
        
        
        boolean twistTurnMode = Robot.JoystickMain.getRawButton(2);
        boolean relativeTurnMode = Robot.JoystickMain.getRawButton(5);
        if(twistTurnMode){
            Robot.rotate(twist);
        }else if (relativeTurnMode){
            Robot.rotate(angularPower);
        }else {
            Robot.move(linearPower);
        }
        
        boolean resetMode = Robot.JoystickMain.getRawButton(6);
        if(resetMode){
            reset();
        }
        
        updateStatus();
    }
    
    /**
     * This function is called periodically during test mode
     */
  
    
    public void updateStatus(){
        SmartDashboard.putDouble("Time Period\t" , m_mainPeriodicTimer.getLastActualPeriod());
        Robot.updateGyroValues();
        SmartDashboard.putDouble("Heading" , Robot.getHeading());
        SmartDashboard.putDouble("Gyro Rate" , Robot.getRate());
    }
    
    public void reset(){
        Robot.gyro.reset();
        Robot.DistanceIntegrator.resetAccumulation();
        Robot.speedIntegrator.resetAccumulation();
        Robot.speedTraveling = 0;
        Robot.init();
    }
    
}
