/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package NerdHerd;
import edu.wpi.first.wpilibj.Timer;

public class LowPassFilter {
        private double m_lastValue = 0;
        private long m_lastTime = -1;
        private double m_RC;
        private Timer timer;
        /**
         * Resets the LowPassFilter.
         */
        public void reset() {
                m_lastValue = 0;
                m_lastTime = -1;
        }

        /**
         * Specifies an RC value and construct a new LowPassFilter object.
         * 
         * @param RC
         */
        public LowPassFilter(double RC) {
                m_RC = RC;
        }

        /**
         * Sets an RC value
         * 
         * @param RC
         */
        public void setRC(double RC) {
                m_RC = RC;
        }

        /**
         * Retrieves an RC value.
         * 
         * @return
         */
        public double getRC() {
                return m_RC;
        }

        /**
         * Call this everytime you need to calculate the value. Recommend you do
         * this every iteration. Otherwise try to reset it.
         * 
         * @param value The value to go in
         * @return The value the filter computes
         */
        public double calculate(double value) {
                if (m_lastTime > 0) {
                        double currentTime = new Timer().get();
                        double a = currentTime - m_lastTime;
                        a /= (a + m_RC);
                        
                        m_lastTime = (long) currentTime;
                        m_lastValue = a * value + (1 - a) * m_lastValue;
                } else
                        m_lastTime = (long) new Timer().get();
                return m_lastValue; 
        }
}
