/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import NerdHerd.NerdyPIDController;
import NerdHerd.Source.NerdyBotThreaded;
import NerdHerd.NerdyCamera;
import NerdHerd.NerdyJoystick;
import edu.wpi.first.wpilibj.camera.AxisCamera;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class ROBOTMAIN extends NerdyBotThreaded {
    
    NerdyPIDController Controller;
    NerdyCamera camera;
    NerdyJoystick JoystickMain;
  
    
    public void robotInit() {
        Controller = new NerdyPIDController();
        Controller.setHeadingTolerance(3);//This is 3 degrees because it drifts 3 degrees / 2.5 minutes
        
        JoystickMain = new NerdyJoystick(1);
        
        //camera = new NerdyCamera(AxisCamera.ResolutionT.k320x240);
        //camera.setPriority(6);
        //camera.start();
        
    }

    public void autonomousPeriodic() {
 
    }

    public void teleopPeriodic() {

        double angle = JoystickMain.get360JoystickAngle();
        double angularPower = Controller.getPIDOutputAngular(angle);
        double linearPower = -JoystickMain.getY();
        double twist = JoystickMain.getTwist();
        
        boolean twistTurnMode = JoystickMain.getRawButton(2);
        boolean relativeTurnMode = JoystickMain.getRawButton(5);
        if(twistTurnMode){
            Controller.rotate(twist);
        }else if (relativeTurnMode){
            Controller.rotate(angularPower);
        }else {
            Controller.move(linearPower);
        }
        
        boolean resetMode = JoystickMain.getRawButton(6);
        if(resetMode){
            Controller.reset();
        }   
    }

}
