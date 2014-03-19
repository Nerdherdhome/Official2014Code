/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package NerdHerd;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *
 * @author student
 */
public class NerdyJoystick extends Joystick {
    
    private double lastJoystickAngle = 0.0;
    public Joystick JoystickMain;
    private final double joystickThreshold = 0.1;
    boolean drive180Mode = false;
    private double joystickBiasY = 0, joystickBiasX = 0;
    
    public NerdyJoystick(int port) {
        super(port);
        JoystickMain = new Joystick(port);
        //joystickBiasY = -JoystickMain.getY();
        //joystickBiasX = JoystickMain.getX();
    }
     public double get360JoystickAngle(){
    
    /*
    Grabs an 360 degree angle reading from the joystick.
    0 degrees is north. 90 degrees is west.
    Should rewrite for arcade drive so that robot always faces forward.
    Should subtract Joystick Bias.
    */
        double y = -JoystickMain.getY()-joystickBiasY;
        double x = -JoystickMain.getX()-joystickBiasX;
        if (Math.abs(x) < joystickThreshold && Math.abs(y) < joystickThreshold ){
            SmartDashboard.putString("Joystick Alert Message", "StopMotor!!!");
            return lastJoystickAngle;
        }
        double angle = (MathUtils.atan2(y,x)) * 180 / Math.PI + 270;
        SmartDashboard.putString("Joystick Alert Message", "Joystick Angle = " + angle);
        lastJoystickAngle = angle;
        if (drive180Mode){
            if (angle >= 90 && angle <= 270){
                angle += 180;//This makes sure that the robot always faces forward.
                angle %= 180;
            }
        }
        return angle%360;
    }
     
     public void toggle180Mode(){
        drive180Mode = !drive180Mode;
     }


    public double getMagnitude(){
        double magnitude = Math.sqrt( sqr(JoystickMain.getY()) + sqr(JoystickMain.getX()));
        magnitude = (magnitude/Math.sqrt(2) ) * -sign(JoystickMain.getY());
        return magnitude; 
     }
    
    public double getXLimited(){
        return 0.0;
    }
    
    public double getYLimited(){
        return 0.0;
    }
    
    private double sqr(double x){
        return x*x;
    }
    
    private int sign(double number){
        if (number > 0){
            return 1;
        }else if (number < 0){
            return -1;
        } else {
            return 0;
        }
    }
}

         
        
     
    

