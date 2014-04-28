/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package nerdHerd.robot2014;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import nerdHerd.util.ThreeCimBallShifter;

/**
 *
 * @author Jordan
 */
public class PainTrain {
    private ThreeCimBallShifter m_leftGearbox, m_rightGearbox;
    private Intake m_intake;
//    private Shooter m_shooter;
    private Compressor m_compressor;
    
    private ThreeCimBallShifter.GearNumber m_gear = ThreeCimBallShifter.GearNumber.kFirstGear;
    private boolean m_isEnabled = false;
    
    
    public PainTrain(int victorIndex, int solenoidIndex){
        m_leftGearbox   = new ThreeCimBallShifter(  new Victor(victorIndex),
                                                    new Victor(victorIndex + 1),
                                                    new Victor(victorIndex + 2),
                                                    new DoubleSolenoid (solenoidIndex,
                                                                        solenoidIndex +1));
                
        m_rightGearbox  = new ThreeCimBallShifter(  new Victor(victorIndex + 3),
                                                    new Victor(victorIndex + 4),
                                                    new Victor(victorIndex + 5));
        
        m_intake        = new Intake( solenoidIndex +2,
                                      solenoidIndex +4,
                                      victorIndex   +6);
        
//        m_shooter       = new Shooter(  canJagIndex +2,
//                                        victorIndex +5,
//                                        victorIndex +6,
//                                        solenoidIndex +6,
//                                        solenoidIndex +7,
//                                        2,
//                                        3
//                                      );
        
        m_compressor    = new Compressor(1,1);
        m_compressor.start();
    }
    
    public void init(){
        m_intake.init();
//        m_shooter.init();
    }
    
    public void disable(){
        m_isEnabled = false;
    }
    
    public void enable(){
        m_isEnabled = true;
        m_intake.enable();
    }
    
    public void run(){
        if(!m_isEnabled){
            m_leftGearbox.set(0.0);
            m_rightGearbox.set(0.0);
            m_intake.disable();
//            m_shooter.disable();
        }
        m_leftGearbox.run();
        m_rightGearbox.run();
        m_intake.run();
//        m_shooter.run();
    }
    
    public void setLeft(double value){
        m_leftGearbox.set(value);
    }
    
    public void setRight(double value){
        m_rightGearbox.set(-value);
    }
    
    public void pullShooter(){
//        m_shooter.pullDown();
    }
    
    public void releaseShooter(){
//        m_shooter.fire();
    }
    
    public void retractIntake(){
        m_intake.raiseArm();
    }
    
    public void extendIntake(){
        m_intake.lowerArm();
    }
    
    public void startIntake(double speed){
        m_intake.setWheelSpeed(speed);
    }
    
    public void stopIntake(){
        m_intake.stopWheels();
    }
    
    public void shift(ThreeCimBallShifter.GearNumber value){
        m_leftGearbox.shift(value);
    }
    
}
