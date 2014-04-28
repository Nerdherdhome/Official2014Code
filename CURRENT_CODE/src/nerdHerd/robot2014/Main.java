/*Badger 4/9/2014
This code is for paintrain without the shooter mechanism.
This code will utilize the intake and drive system.
This is based on having all talons
*/
package nerdHerd.robot2014;

import edu.wpi.first.wpilibj.Joystick;
import nerdHerd.util.NerdyBot;
import nerdHerd.util.NerdyTimer;
import nerdHerd.util.ThreeCimBallShifter;

public class Main extends NerdyBot{
    Joystick leftJoystick, rightJoystick, articJoystick;
    NerdyTimer autonomousTimer = new NerdyTimer(2.0);
    PainTrain myRobot;
    
    double leftSpeed = 0.0, rightSpeed = 0.0;
    boolean isRunningAutonomous = false;
    

    public void robotInit(){
        myRobot         = new PainTrain(1,1);
        leftJoystick    = new Joystick(1);
        rightJoystick   = new Joystick(2);
        articJoystick   = new Joystick(3);
        autonomousTimer.start();
    }

    public void autonomousPeriodic() {
    }
    
    public void autonomousContinous(){
        if(isRunningAutonomous){
            myRobot.setLeft(-1.0);
            myRobot.setRight(-1.0);
            isRunningAutonomous = !autonomousTimer.hasPeriodPassed();
        }else{
            myRobot.setLeft(0.0);
            myRobot.setRight(0.0);            
        }
        myRobot.run();
//        theIntake.run();    
    }
    
    public void autonomousInit(){
        myRobot.enable();
        autonomousTimer.reset();
        myRobot.shift(ThreeCimBallShifter.GearNumber.kFirstGear);
        isRunningAutonomous = true;
//        theIntake.init();
    }
    
    public void teleopInit() {
        myRobot.enable();
    }

    public void teleopPeriodic() {
        leftSpeed           =  leftJoystick.getY();
        rightSpeed          =  rightJoystick.getY();
        boolean shiftUp     =  leftJoystick.getRawButton(3);
        boolean shiftDown   =  leftJoystick.getRawButton(2);
        boolean intakeIn    =  articJoystick.getRawButton(11);
        boolean intakeOut   =  articJoystick.getRawButton(10);
        boolean intake      =  articJoystick.getRawButton(3);
        boolean outTake     =  articJoystick.getRawButton(2);
        
        if(intakeIn){
            myRobot.retractIntake();
        }else if(intakeOut){
            myRobot.extendIntake();
        }

        if(intake){
            myRobot.startIntake(1.0);
        }else if(outTake){
            myRobot.startIntake(-1.0);
        }else{
            myRobot.stopIntake();
        }
        
        if(shiftUp){
            myRobot.shift(ThreeCimBallShifter.GearNumber.kSecondGear);
        }else if(shiftDown){
            myRobot.shift(ThreeCimBallShifter.GearNumber.kFirstGear);
        }
        myRobot.   setLeft(leftSpeed);
        myRobot.   setRight(rightSpeed);
    }
       
    public void teleopContinous(){
        myRobot.run();
    }
    
    public void disabledInit(){
        leftSpeed   = 0.0;
        rightSpeed  = 0.0;
        myRobot.disable();
    }
    
    public void disabled(){
        myRobot.run();
    }
}
