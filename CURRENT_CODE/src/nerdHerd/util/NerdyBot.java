/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package nerdHerd.util;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.communication.FRCControl;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


/**
 * IterativeRobot implements a specific type of Robot Program framework, extending the RobotBase class.
 *
 * robotInit() -- provide for initialization at robot power-on
 *
 * init() functions -- each of the following functions is called once when the
 *                     appropriate mode is entered:
 *  - DisabledInit()   -- called only when first disabled
 *  - AutonomousInit() -- called each and every time autonomous is entered from another mode
 *  - TeleopInit()     -- called each and every time teleop is entered from another mode
 *  - TestInit()       -- called each and every time test mode is entered from anothermode
 *
 * Periodic() functions -- each of these functions is called iteratively at the
 *                         appropriate periodic rate (aka the "slow loop").  The period of
 *                         the iterative robot is synced to the driver station control packets,
 *                         giving a periodic frequency of about 50Hz (50 times per second).
 *   - disabledPeriodic()
 *   - autonomousPeriodic()
 *   - teleopPeriodic()
 *   - testPeriodoc()
 *
 */
public class NerdyBot extends RobotBase {

    private boolean m_disabledInitialized;
    private boolean m_autonomousInitialized;
    private boolean m_teleopInitialized;
    private boolean m_testInitialized;
    private NerdyTimer m_fastLoopTimer;
    /**
     * Constructor for NerdyBot
     *
     * The constructor initializes the instance variables for the robot to indicate
     * the status of initialization for disabled, autonomous, and teleop code.
     */
    public NerdyBot() {
        // set status for initialization of disabled, autonomous, and teleop code.
        m_disabledInitialized   = false;
        m_autonomousInitialized = false;
        m_teleopInitialized     = false;
        m_testInitialized       = false;
        m_fastLoopTimer         = new NerdyTimer(.03);
        m_fastLoopTimer.start();
    }

    /**
     * Provide an alternate "main loop" via startCompetition().
     *
     */
    public void startCompetition() {
        UsageReporting.report(UsageReporting.kResourceType_Framework, UsageReporting.kFramework_Iterative);
        robotInit();
        LiveWindow.setEnabled(false);
        while (true) {
            if (isDisabled()) {
                if (!m_disabledInitialized) {
                    LiveWindow.setEnabled(false);
                    disabledInit();
                    m_disabledInitialized   = true;
                    m_autonomousInitialized = false;
                    m_teleopInitialized     = false;
                    m_testInitialized       = false;
                    m_fastLoopTimer.reset();
                }
                if(m_fastLoopTimer.hasPeriodPassed()){
                    disabledContinous();
                    if (nextPeriodReady()) {
                        FRCControl.observeUserProgramDisabled();
                        disabledPeriodic();
                    }
                }
            } else if (isTest()) {
                if (!m_testInitialized) {
                    LiveWindow.setEnabled(true);
                    testInit();
                    m_testInitialized       = true;
                    m_autonomousInitialized = false;
                    m_teleopInitialized     = false;
                    m_disabledInitialized   = false;
                    m_fastLoopTimer.reset();                    
                }
                if(m_fastLoopTimer.hasPeriodPassed()){
                    testContinous();
                    if (nextPeriodReady()) {
                        FRCControl.observeUserProgramTest();
                        testPeriodic();
                    }
                }
            } else if (isAutonomous()) {
                if (!m_autonomousInitialized) {
                    LiveWindow.setEnabled(false);
                    autonomousInit();
                    m_autonomousInitialized = true;
                    m_testInitialized       = false;
                    m_teleopInitialized     = false;
                    m_disabledInitialized   = false;
                    m_fastLoopTimer.reset();                    
                }
                if(m_fastLoopTimer.hasPeriodPassed()){
                    autonomousContinous();
                    if (nextPeriodReady()) {
                        getWatchdog().feed();
                        FRCControl.observeUserProgramAutonomous();
                        autonomousPeriodic();
                    }
                }
            } else {
                if (!m_teleopInitialized) {
                    LiveWindow.setEnabled(false);
                    teleopInit();
                    m_teleopInitialized     = true;
                    m_testInitialized       = false;
                    m_autonomousInitialized = false;
                    m_disabledInitialized   = false;
                    m_fastLoopTimer.reset();
                }
                if(m_fastLoopTimer.hasPeriodPassed()){
                    teleopContinous();
                    if (nextPeriodReady()) {
                        getWatchdog().feed();
                        FRCControl.observeUserProgramTeleop();
                        teleopPeriodic();
                    }
                }
            }
        }
    }

    /**
     * Determine if the appropriate next periodic function should be called.
     * Call the periodic functions whenever a packet is received from the Driver Station, or about every 20ms.
     */
    private boolean nextPeriodReady() {
        return m_ds.isNewControlData();
    }

    /* ----------- Overridable initialization code -----------------*/

    /**
     * Robot-wide initialization code should go here.
     *
     * Users should override this method for default Robot-wide initialization which will
     * be called when the robot is first powered on.  It will be called exactly 1 time.
     */
    public void robotInit() {
        System.out.println("Default robotInit()");
    }

    /**
     * Initialization code for disabled mode should go here.
     *
     * Users should override this method for initialization code which will be called each time
     * the robot enters disabled mode.
     */
    public void disabledInit() {
        System.out.println("Default disabledInit()");
    }

    /**
     * Initialization code for autonomous mode should go here.
     *
     * Users should override this method for initialization code which will be called each time
     * the robot enters autonomous mode.
     */
    public void autonomousInit() {
        System.out.println("Default autonomousInit()");
    }

    /**
     * Initialization code for teleop mode should go here.
     *
     * Users should override this method for initialization code which will be called each time
     * the robot enters teleop mode.
     */
    public void teleopInit() {
        System.out.println("Default teleopInit()");
    }
    
    /**
     * Initialization code for test mode should go here.
     * 
     * Users should override this method for initialization code which will be called each time
     * the robot enters test mode.
     */
    public void testInit() {
        System.out.println("Default testInit()");
    }

   /* ----------- Overridable periodic code -----------------*/

    /**
     * Periodic code for disabled mode should go here.
     *
     * Users should override this method for code which will be called periodically at a regular
     * rate while the robot is in disabled mode.
     */
    public void disabledPeriodic() {
 
    }
    
    /**
     * Periodic code for autonomous mode should go here.
     *
     * Users should override this method for code which will be called periodically at a regular
     * rate while the robot is in autonomous mode.
     */
    public void autonomousPeriodic() {

    }


    /**
     * Periodic code for teleop mode should go here.
     *
     * Users should override this method for code which will be called periodically at a regular
     * rate while the robot is in teleop mode.
     */
    public void teleopPeriodic() {

    }
    
    
    /**
     * Periodic code for test mode should go here
     * 
     * Users should override this method for code which will be called periodically at a regular rate
     * while the robot is in test mode.
     */
    public void testPeriodic() {

    }

    public void teleopContinous(){

    }
    
    public void autonomousContinous(){
        
    }

    private void testContinous() {
        
    }

    private void disabledContinous() {
        
    }
}
