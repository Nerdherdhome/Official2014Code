/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package nerdHerd.util;


import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author student
 */
public class ThreeCimBallShifter {
    
public static class GearNumber{
    public final int value;
    static final int kFirstGear_val = 1;
    static final int kSecondGear_val = 2;
    
    public static final GearNumber kFirstGear = new GearNumber(kFirstGear_val);
    public static final GearNumber kSecondGear = new GearNumber(kSecondGear_val);
    
    
    private GearNumber(int value){
        this.value = value;
    }
}

/*Class objects

*/
private final SpeedController   m_firstController,
                                m_secondController,
                                m_thirdController;
private final DoubleSolenoid m_shifterSolenoid;
private final NerdyTimer m_delaySolenoidOnTime = new NerdyTimer(.05);
/*Class variables

*/
private GearNumber  m_gearShiftTo   = GearNumber.kFirstGear,
                    m_gearIn        = GearNumber.kSecondGear;
private double      m_shiftCurrent  = 0.0;
private double      m_speedValue    = 0.0;


/* Control Structure

*/
private boolean m_canCurrentShift   = false;
private boolean m_canYouEvenShift   = false;
private boolean m_isSolenoidOn      = false;

public ThreeCimBallShifter( SpeedController main,
                            SpeedController sub1,
                            SpeedController sub2){
    m_firstController   = main;
    m_secondController  = sub1;
    m_thirdController   = sub2;
    m_shifterSolenoid   = null;
}

public ThreeCimBallShifter( SpeedController main,
                            SpeedController sub1,
                            SpeedController sub2,
                            DoubleSolenoid  sol){
    m_firstController   = main;
    m_secondController  = sub1;
    m_thirdController   = sub2;
    m_shifterSolenoid   = sol;
    m_canYouEvenShift   = true;
    m_delaySolenoidOnTime.start();
}


public void shift(GearNumber value){
    m_gearShiftTo = value;
}

public void set(double value){
    m_speedValue = value;
}

public void enableCurrentBasedShift(double value){
    m_shiftCurrent      = value;
    m_canCurrentShift   = true;
}

public void disableCurrentBasedShift(){
    m_canCurrentShift   = false;
}

public void run(){
    m_run();
}

private void m_run(){
    try{
    m_firstController   .set(m_speedValue);
    m_secondController  .set(m_speedValue);
    m_thirdController   .set(m_speedValue);
//    m_secondVictor.set(m_speedValue);
    }catch (Exception e){
        
    }
    
    /*Shift code
    
    */
    if(m_canYouEvenShift){
        if(m_gearShiftTo != m_gearIn){
            if(ThreeCimBallShifter.GearNumber.kFirstGear == m_gearShiftTo){
                m_shifterSolenoid.set(DoubleSolenoid.Value.kReverse);
            }else{
                m_shifterSolenoid.set(DoubleSolenoid.Value.kForward);
            }
            m_gearIn = m_gearShiftTo;
            m_delaySolenoidOnTime.reset();
            m_isSolenoidOn = true;
        }else{
            if(m_isSolenoidOn){
                if(m_delaySolenoidOnTime.hasPeriodPassed()){
                    m_isSolenoidOn = false;
                    m_shifterSolenoid.set(DoubleSolenoid.Value.kOff);
                }
            }
        }
    }
}

 
}
