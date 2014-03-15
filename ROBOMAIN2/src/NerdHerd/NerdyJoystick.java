/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package NerdHerd;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Joystick;
/**
 *
 * @author student
 */
public class NerdyJoystick extends Joystick {
    
    private double lastJoystickAngle = 0.0;
    public Joystick JoystickMain;
    private double joystickThreshold = 0.1;
    boolean drive180Mode = false;
    
    public NerdyJoystick(int port) {
        super(port);
        JoystickMain = new Joystick(port);
    }
     public double get360JoystickAngle(){
    
    /*
    Grabs an 360 degree angle reading from the joystick.
    0 degrees is north. 90 degrees is west.
    Should rewrite for arcade drive so that robot always faces forward.
    Should subtract Joystick Bias.
    */
    
        double y = -JoystickMain.getY();
        double x = -JoystickMain.getX();
            if (Math.abs(x) < joystickThreshold && Math.abs(y) < joystickThreshold ){
                //need heading update code
                return lastJoystickAngle;
                }
            double angle = (MathUtils.atan2(y,x)) * 180 / Math.PI + 270;
        if (drive180Mode){
            if (angle >= 90 && angle <= 270){
                angle += 180;//This makes sure that the robot always faces forward.
                angle %= 180;
            }
        }
        return angle%360;
    }
     
     public void toggle180Mode(){
         if(drive180Mode = false){
             drive180Mode = true;
         }else{
             drive180Mode = false;
         }
         
         }
     }

         
        
     
    

