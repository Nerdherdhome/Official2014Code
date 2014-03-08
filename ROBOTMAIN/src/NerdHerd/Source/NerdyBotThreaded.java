/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package NerdHerd.Source;

import com.sun.squawk.GC;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.communication.FRCControl;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import java.lang.Runnable;


/**
 * IterativeRobot implements a specific type of Robot Program framework, extending the RobotBase class.
 *
 * The IterativeRobot class is intended to be subclassed by a user creating a robot program.
 *
 * This class is intended to implement the "old style" default code, by providing
 * the following functions which are called by the main loop, startCompetition(), at the appropriate times:
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
 * Periodic() functions -- 
 *   - disabledPeriodic()
 *   - autonomousPeriodic()
 *   - teleopPeriodic()
 *   - testPeriodoc()
 *
 */
public class NerdyBotThreaded extends RobotBase implements Runnable{

    private final static boolean TRACE_LOOP_ALLOCATIONS = false; // master tracing switch
    private final static boolean TRACE_LOOP_ALLOCATIONS_AFTER_INIT = false;  // trace before or after all phases initialize
    protected NerdyTimer m_mainPeriodicTimer;
    private boolean m_disabledInitialized;
    private boolean m_autonomousInitialized;
    private boolean m_teleopInitialized;
    private boolean m_testInitialized;
    public final static double k_PeriodTime = 0.04;
    private boolean disabledPacketHandled = false;
    private boolean autonomousPacketHandled = false;
    private boolean teleopPacketHandled = false;
    private boolean testPacketHandled = false;
    private final int TRACE_LOOP_MAX = 100;
    private int loopCount = TRACE_LOOP_MAX;
   
        
    public void run() {
        // tracing support:



        m_mainPeriodicTimer.start();
        while (true) {
            double startTime = getSeconds();
            if (TRACE_LOOP_ALLOCATIONS && disabledPacketHandled && autonomousPacketHandled && teleopPacketHandled && --loopCount <= 0) {
                System.out.println("!!!!! Stop loop!");
                break;
            }
            // Call the appropriate function depending upon the current robot mode
            if (isDisabled()) {
                // call DisabledInit() if we are now just entering disabled mode from
                // either a different mode or from power-on
                if (!m_disabledInitialized) {
                    LiveWindow.setEnabled(false);
                    disabledInit();
                    m_disabledInitialized = true;
                    // reset the initialization flags for the other modes
                    m_autonomousInitialized = false;
                    m_teleopInitialized = false;
                    m_testInitialized = false;
                }
                
                //will handle packets only after main control loop has passed
                disabledPeriodic();
                if (nextPacketReady()) {
                    FRCControl.observeUserProgramDisabled();
                    disabledHandlePacket();
                    disabledPacketHandled = true;
                }
            } else if (isTest()) {
                // call TestInit() if we are now just entering test mode from either
                // a different mode or from power-on
                if (!m_testInitialized) {
                    LiveWindow.setEnabled(true);
                    testInit();
                    m_testInitialized = true;
                    m_autonomousInitialized = false;
                    m_teleopInitialized = false;
                    m_disabledInitialized = false;
                }

                //will handle packets only after main control loop has passed
                testPeriodic();
                if (nextPacketReady()) {
                    FRCControl.observeUserProgramTest();
                    testHandlePacket();
                    testPacketHandled = true;
                }
                
            } else if (isAutonomous()) {
                // call Autonomous_Init() if this is the first time
                // we've entered autonomous_mode
                if (!m_autonomousInitialized) {
                    LiveWindow.setEnabled(false);
                    // KBS NOTE: old code reset all PWMs and relays to "safe values"
                    // whenever entering autonomous mode, before calling
                    // "Autonomous_Init()"
                    autonomousInit();
                    m_autonomousInitialized = true;
                    m_testInitialized = false;
                    m_teleopInitialized = false;
                    m_disabledInitialized = false;
                }

                //will handle packets only after main control loop has passed
                autonomousPeriodic();
                if (nextPacketReady()) {
                    getWatchdog().feed();
                    FRCControl.observeUserProgramAutonomous();
                    autonomousHandlePacket();
                    autonomousPacketHandled = true;
                }

                
            } else {
                // call Teleop_Init() if this is the first time
                // we've entered teleop_mode
                if (!m_teleopInitialized) {
                    LiveWindow.setEnabled(false);
                    teleopInit();
                    m_teleopInitialized = true;
                    m_testInitialized = false;
                    m_autonomousInitialized = false;
                    m_disabledInitialized = false;
                }


                //will handle packets only after main control loop has passed
                teleopPeriodic();
                if (nextPacketReady()) {
                    getWatchdog().feed();
                    FRCControl.observeUserProgramTeleop();
                    teleopHandlePacket();
                    teleopPacketHandled = true;                    
                }
                
            }

            double sleepTime = k_PeriodTime-(getSeconds()-startTime);//seconds

            try{
                Thread.sleep((long)(sleepTime*1000));//milliseconds
            }catch(Exception e){
                System.out.println(e);
                break;
            }
        }
    }
    /**
     * Return the system clock time in seconds. Return the time from the
     * FPGA hardware clock in seconds since the FPGA started.
     *
     * @return Robot running time in seconds.
     */
    private double getSeconds(){
        return  Utility.getFPGATime() / 1000000.0;
    }
    /**
     * Constructor for RobotIterativeBase
     *
     * The constructor initializes the instance variables for the robot to indicate
     * the status of initialization for disabled, autonomous, and teleop code.
     */
    public NerdyBotThreaded() {
        // set status for initialization of disabled, autonomous, and teleop code.
        m_disabledInitialized = false;
        m_autonomousInitialized = false;
        m_teleopInitialized = false;
        m_testInitialized = false;
        m_mainPeriodicTimer = new NerdyTimer(k_PeriodTime);
    }

    /**
     * Provide an alternate "main loop" via startCompetition().
     *
     */
    public void startCompetition() {
        UsageReporting.report(UsageReporting.kResourceType_Framework, UsageReporting.kFramework_Iterative);

        robotInit();
        Object marker = null;

        if (TRACE_LOOP_ALLOCATIONS) {
            GC.initHeapStats();
            if (!TRACE_LOOP_ALLOCATIONS_AFTER_INIT) {
                System.out.println("=== Trace allocation in competition loop! ====");
                marker = new Object(); // start counting objects before any loop starts - includes class initialization
            }
        }
  
        LiveWindow.setEnabled(false);
      
        //start all threads
        Thread p = new Thread(this,"Control Thread");
        p.setPriority(ROBOT_TASK_PRIORITY);
        p.start();
        //TODO Start the vision processing at the ROBOT_TASK_PRIORITY-1
        //TODO Start the smartdashboard reporting processing at ROBOT_TASK_PRIORITY-2
        try{
            p.join();//This blocks until the thread dies
        }catch(Exception e){
            System.out.println(e);
        }

        
        
        if (TRACE_LOOP_ALLOCATIONS) {
            GC.printHeapStats(marker, false);
        }
        if (TRACE_LOOP_ALLOCATIONS && TRACE_LOOP_ALLOCATIONS_AFTER_INIT &&
                disabledPacketHandled && autonomousPacketHandled && teleopPacketHandled && loopCount == TRACE_LOOP_MAX) {
            System.out.println("=== Trace allocation in competition loop! ====");
            marker = new Object(); // start counting objects after 1st loop completes - ignore class initialization
        }
        
    }

    

    /**
     * Determine if the appropriate next periodic function should be called.
     * Call the periodic functions whenever a packet is received from the Driver Station, or about every 20ms.
     */
    private boolean nextPacketReady() {
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

    private boolean dpFirstRun = true;
    /**
     * Periodic code for disabled mode should go here.
     *
     * Users should override this method for code which will be called periodically at a regular
     * rate while the robot is in disabled mode.
     */
    public void disabledPeriodic() {
        if (dpFirstRun) {
            System.out.println("Default disabledPeriodic()");
            dpFirstRun = false;
        }
    }

    private boolean apFirstRun = true;

    /**
     * Periodic code for autonomous mode should go here.
     *
     * Users should override this method for code which will be called periodically at a regular
     * rate while the robot is in autonomous mode.
     */
    public void autonomousPeriodic() {
        if (apFirstRun) {
            System.out.println("Default autonomousPeriodic()");
            apFirstRun = false;
        }
    }

    private boolean tpFirstRun = true;

    /**
     * Periodic code for teleop mode should go here.
     *
     * Users should override this method for code which will be called periodically at a regular
     * rate while the robot is in teleop mode.
     */
    public void teleopPeriodic() {
        if (tpFirstRun) {
            System.out.println("Default teleopPeriodic()");
            tpFirstRun = false;
        }
    }
    
    private boolean tmpFirstRun = true;
    
    /**
     * Periodic code for test mode should go here
     * 
     * Users should override this method for code which will be called periodically at a regular rate
     * while the robot is in test mode.
     */
    public void testPeriodic() {
        if (tmpFirstRun) {
            System.out.println("Default testPeriodic()");
            tmpFirstRun = false;
        }
    }

    public void teleopHandlePacket(){
        if (tmpFirstRun) {
            System.out.println("Default teleopHandlePacket()");
            tmpFirstRun = false;
        }
    }
    
    public void autonomousHandlePacket(){
        if (tmpFirstRun) {
            System.out.println("Default autonomousHandlePacket()");
            tmpFirstRun = false;
        }
    }
    
    public void testHandlePacket(){
        if (tmpFirstRun) {
            System.out.println("Default testHandlePacket()");
            tmpFirstRun = false;
        }
    }

    public void disabledHandlePacket() {
        if (tmpFirstRun) {
            System.out.println("Default disabledHandlePacket()");
            tmpFirstRun = false;
        }    
    }
}
