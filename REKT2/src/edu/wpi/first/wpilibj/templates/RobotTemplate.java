/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import NerdHerd.NerdySensors;
import Nerdherd.NerdyJoystick;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    private CANJaguar ltMain, ltSub1, ltSub2, rtMain, rtSub1, rtSub2;
    private NerdyJoystick leftStick, rightStick;
    private DoubleSolenoid solenoid;
    private Compressor roboComp;
    private Solenoid awesome;
    boolean isInHighGear = false; 
    boolean lastVal = false;
    int loopWait = 0;
    private NerdySensors Sensor;
    
    public void robotInit() {
            leftStick = new NerdyJoystick(1);
            rightStick = new NerdyJoystick(2);
            solenoid = new DoubleSolenoid(1,2);
            roboComp = new Compressor(1, 1);
            roboComp.start();
            awesome = new Solenoid(3);
        try{
            ltMain = new CANJaguar(2);
            ltSub1 = new CANJaguar(4);
            ltSub2 = new CANJaguar(6);
            rtMain = new CANJaguar(3);
            rtSub1 = new CANJaguar(5);
            rtSub2 = new CANJaguar(7);
            Sensor = new NerdySensors();
            Sensor.setPriority(4);
            Sensor.start();
        }catch (Exception e){
            System.out.println(e);
        }
                System.out.println("I am uploading!");
    }
    public void teleopInit(){
//        roboComp.start();
        System.out.println("I am uploading!");
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
            double leftDrive = leftStick.getYLimited();
            double rightDrive = rightStick.getYLimited();
            double leftDriveActual = leftStick.getY();
            boolean isTank  = (leftStick.getRawAxis(3) > .5);
            System.out.println("Final Value: " + leftDrive + " Actual: " + leftDriveActual);
            System.out.println("Tank drive" + isTank);
            if(isTank){
                try{
                    ltMain.set(leftDrive);
                    ltSub1.set(leftDrive);
                    ltSub2.set(leftDrive);
                    rtMain.set(-rightDrive);
                    rtSub1.set(-rightDrive);
                    rtSub2.set(-rightDrive);
                }
                catch (Exception e){
                    System.out.println(e);
                }
            }
        boolean shiftUp = leftStick.getRawButton(3);
        boolean shiftDown = leftStick.getRawButton(2);
        if(shiftUp){
            solenoid.set(DoubleSolenoid.Value.kReverse);
            loopWait = 5;
        } else if(shiftDown){
            solenoid.set(DoubleSolenoid.Value.kForward);
            loopWait = 5;
        }else{
            if(0< loopWait){
                loopWait--;
          }else{
                loopWait = 0;
                solenoid.set(DoubleSolenoid.Value.kOff);
            }
        }
        awesome.set(roboComp.getPressureSwitchValue());
        roboComp.start();
    }
    
    /**
     * This function is called periodically during test mode
     */
   
    
    public void testPeriodic() {
    
    }
    
}
