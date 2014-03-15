/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package NerdHerd.Source;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author student
 */
public class NerdyTimer extends Timer {
    
    private double m_lastActualPeriod;
    
    public NerdyTimer(double period){
        m_period = period;
    }
    
    public boolean hasPeriodPassed(){
        m_lastActualPeriod = get();
        if(m_lastActualPeriod >= m_period){
            reset();
            return true;
        }else{
            return false;
        }
    }
    
    private double m_period;
    
    public double getLastActualPeriod(){
        return m_lastActualPeriod;
    }
}
