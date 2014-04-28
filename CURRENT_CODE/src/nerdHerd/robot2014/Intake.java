/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package nerdHerd.robot2014;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import nerdHerd.util.NerdyTimer;

/**
 *
 * @author Jordan
 */
public class Intake {
    private DoubleSolenoid m_leftSolenoid, m_rightSolenoid;
    private Victor m_intakeMotor;
    private NerdyTimer m_solenoidTime;
    
    private DoubleSolenoid.Value m_solenoidValue    = DoubleSolenoid.Value.kOff;
    private double  m_speed                         = 0.0;
    private boolean m_isEnabled                     = false;
    private boolean m_isSolenoidActive              = false;

    
    Intake(int solenoid1Port1, int solenoid2Port1, int victorNumber){
        m_leftSolenoid  = new DoubleSolenoid(solenoid1Port1, solenoid1Port1 +1);
        m_rightSolenoid = new DoubleSolenoid(solenoid2Port1, solenoid2Port1 +1);
        m_intakeMotor   = new Victor(victorNumber);
        m_solenoidTime  = new NerdyTimer(.05);
        m_solenoidTime.start();
    }
    
    public void init(){
        m_solenoidValue     = DoubleSolenoid.Value.kReverse;
        m_solenoidTime.reset();
        m_isSolenoidActive  = true;
    }
    
    public void lowerArm(){
        m_solenoidValue     = DoubleSolenoid.Value.kForward;
        m_solenoidTime.reset();
        m_isSolenoidActive  = true;
    }
    
    public void raiseArm(){
        m_solenoidValue     = DoubleSolenoid.Value.kReverse;
        m_solenoidTime.reset();
        m_isSolenoidActive  = true;
    }
    
    public void setWheelSpeed(double speed){
        m_speed = speed;
    }
    
    public void stopWheels(){
        m_speed = 0.0;
    }
    
    public void enable(){
        m_isEnabled = true;
        m_speed = 0.0;
    }
    
    public void disable(){
        m_isEnabled = false;
        m_speed = 0.0;
    }
    
    public void run(){
        if(m_isEnabled){
            if(m_isSolenoidActive){
                if(m_solenoidTime.hasPeriodPassed()){
                    m_isSolenoidActive = false;
                    m_solenoidValue    = DoubleSolenoid.Value.kOff;
                }
            }
            m_leftSolenoid. set(m_solenoidValue);
            m_rightSolenoid.set(m_solenoidValue);
            m_intakeMotor.set(m_speed);
            System.out.println(m_speed);
        }else{
            m_leftSolenoid. set(DoubleSolenoid.Value.kOff);
            m_rightSolenoid.set(DoubleSolenoid.Value.kOff);
            m_intakeMotor.set(0.0);                        
        }
    }
}
