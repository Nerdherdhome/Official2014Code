/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package nerdHerd.robot2014;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import java.lang.Math;
import nerdHerd.util.NerdyTimer;

/**
 *
 * @author Jordan
 */
public class Shooter {
    private CANJaguar m_jag;
    private Victor  m_victorOne, m_victorTwo;
    private DoubleSolenoid m_releaseSol;
    private DigitalInput m_lowerSwitch;
    private NerdyTimer m_solenoidDeadTime, m_shooterDelay;
    private Encoder m_pullDownEncoder;
    
    private DoubleSolenoid.Value m_solValue = DoubleSolenoid.Value.kReverse;
    private double  m_valToMotors           = 0.0;
    private final double m_backSpin         = 1;
    private boolean m_isEnabled             = false;
    private boolean m_isArmed               = false;
    private boolean m_pullDown              = false;
    private boolean m_fire                  = false;
    private boolean m_isPulledDown          = false;
    private boolean m_isSlacked             = false;
    private boolean m_isSolenoidActive      = false;
    
    public Shooter(int jag, 
                   int victor1, 
                   int victor2, 
                   int sol1, 
                   int sol2, 
                   int btn, 
                   int encoderNum1 ){
        try{
            m_jag               = new CANJaguar(jag);
            m_victorOne         = new Victor(victor1);
            m_victorTwo         = new Victor(victor2);
            m_releaseSol        = new DoubleSolenoid(sol1,sol2);
            m_lowerSwitch       = new DigitalInput(btn);
            m_solenoidDeadTime  = new NerdyTimer(0.05);
            m_shooterDelay      = new NerdyTimer(2);
            m_pullDownEncoder   = new Encoder(encoderNum1, encoderNum1 +1);
            m_pullDownEncoder.setDistancePerPulse(.72);
            m_pullDownEncoder.start();
            m_solenoidDeadTime.start();
            m_shooterDelay.start();
            }catch(CANTimeoutException e){}
    }
    
    public void init(){
        m_solValue              = DoubleSolenoid.Value.kReverse;
        m_solenoidDeadTime.reset();
    }
    
    public void run(){
        if(!m_isEnabled){
            m_solValue              = DoubleSolenoid.Value.kOff;
            m_valToMotors           = 0.0;
        }else if(m_isSlacked){
            if(m_fire){
                m_solValue          = DoubleSolenoid.Value.kReverse;
                m_solenoidDeadTime.reset();
                m_fire              = false;
                m_isArmed           = false;
                m_isSlacked         = false;
                m_pullDown          = false;
            }
        }else if(m_isArmed){ 
            if(m_backSpin <= Math.abs(m_pullDownEncoder.getDistance())){
                m_valToMotors       = 0.0;
                m_isArmed           = true;
                m_isSlacked         = true;
            }
        }else if(m_pullDown){
            if(!m_isPulledDown){
                m_isPulledDown      = m_lowerSwitch.get();
                m_valToMotors       = -.5;
            }else{
                m_pullDown          = false;
                m_isSolenoidActive  = false;
                m_isArmed           = true;
                m_valToMotors       = 0.0;
                m_solValue          = DoubleSolenoid.Value.kForward;
                m_solenoidDeadTime.reset();
                m_pullDownEncoder.reset();
            }
        }
        
        try{
            m_jag.setX(m_valToMotors);
            m_victorOne.set(m_valToMotors);
            m_victorTwo.set(m_valToMotors);
        }catch(CANTimeoutException e){}
        if(m_isSolenoidActive){
            if(m_solenoidDeadTime.hasPeriodPassed()){
                m_isSolenoidActive = false;
            }         
        }
        
    }
    
    public void enable(){
        m_isEnabled = true;
    }
    
    public void disable(){
        m_isEnabled = true;
    }
    
    public void pullDown(){
        if(m_pullDown != true){
            m_pullDown = true && m_shooterDelay.hasPeriodPassed();
        }
    }
    
    public boolean isArmed(){
        return m_isArmed;
    }
    
    public void fire(){
        m_fire = true && m_isSlacked;
    }
    
    
}
