/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package NerdHerd;

import NerdHerd.Source.NerdyBotThreaded;
import com.sun.cldc.jna.Pointer;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NerdyCamera extends Thread{

    //final int RECTANGULARITY_LIMIT = 50;
    //final int ASPECT_RATIO_LIMIT = 65;
    AxisCamera camera = AxisCamera.getInstance();
    CriteriaCollection cc;
    final int AREA_MINIMUM = 300;//Original was 150. Changed to 150 to support greater distances.
    private int imageWidth = 320, imageHeight = 240;
    final int MAX_PARTICLES = 8;
    private Pointer imagePointer;
    private ColorImage image = null;
    private BinaryImage thresholdImage = null, filteredImage = null;
    private BinaryImage objects = null;
    private int highestScoreParticleNum = 0;
    private int top = 0, left = 0, height = 0, width = 0;
    private double areaPercent = 0, distance = 0, angle = 0;
    public int desiredAreaPercent = 75;
    public int maxDistance = 13, minDistance = 8;
    private final double actualWidth = 2;//Feet
    private NerdyCameraData cameraData;
    public boolean cameraActive = true, dataReportingActive = true;;
    
     
    public NerdyCamera(AxisCamera.ResolutionT resolution) {
        camera.writeResolution(resolution);
        cc = new CriteriaCollection();
        cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        imageWidth = camera.getResolution().width;
        imageHeight = camera.getResolution().height;
        
        
        cameraData = new NerdyCameraData();
        cameraData.setPriority(1);
        cameraData.start();
    }
    
    public class Scores{
        double rectangularity;
        double aspectRatioHorizontal;
        double aspectRatioVertical;
    }
    
    public void run() {
        while(cameraActive){
            double startTime = getSeconds();
            
            image = null;
            thresholdImage = null;
            filteredImage = null;
            objects = null;
            try{
                image = camera.getImage();
                //thresholdImage = image.thresholdHSV(135, 170, 85, 100, 100, 100); //green filter
                //thresholdImage = image.thresholdHSV(18, 27, 40, 255, 90, 255); //orange filter
                thresholdImage = image.thresholdRGB(150, 255, 150, 255, 50, 150); //orange filter
                thresholdImage.write("/camera.bmp");

                objects = thresholdImage.removeLargeObjects(true, AREA_MINIMUM);
                objects.write("/lose.bmp");

                filteredImage = objects.particleFilter(cc);
                filteredImage.write("/test.bmp");     //test if it is filtering objects

                Scores scores[] = new Scores[filteredImage.getNumberParticles()];
                
                //Consider Removing
                if(filteredImage.getNumberParticles() > 0){
                   for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles();i++) {
                       ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                       scores[i] = new Scores();
                       //score if it can score
                       scores[i].rectangularity = scoreRectangularity(report);
                       scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
                       scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false); 
                   }
                }

                imagePointer = filteredImage.image;
                NIVision.particleFilter(imagePointer, imagePointer, cc);
                
                int particles = NIVision.countParticles(imagePointer);
                highestScoreParticleNum = 0;
                for(int i = 0; i < particles; i++){
                    int area1 = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_AREA);
                    int area2 = (int)NIVision.MeasureParticle(imagePointer, i, false, MeasurementType.IMAQ_MT_AREA);
                    if(area2 > area1){
                        highestScoreParticleNum = i;
                    }
                }

                top = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_TOP);
                left = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_LEFT);
                height = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT);
                width = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
                areaPercent = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_AREA);
                areaPercent /= (width*height)/100;
                distance = actualWidth*imageWidth/(actualWidth*width*Math.tan(0.4101523));//Math.tan(47.7/2 degrees)
                angle = (47 / imageWidth)*(int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_CENTER_OF_MASS_X);
                
            } catch (Exception e) {
                System.out.println(e);
            }finally{
                try{
                    thresholdImage.free();
                    objects.free();
                    filteredImage.free();
                    image.free();
                    imagePointer.free();
                }catch (Exception ex) {
                    System.out.println(ex);
                }
            }
            
            double timePassed = getSeconds()-startTime;
            double sleepTime = NerdyBotThreaded.k_PeriodTime-(timePassed);//seconds
            SmartDashboard.putDouble("TimePassedSensor", timePassed);
            SmartDashboard.putDouble("SleepTimeSensor", sleepTime);
            if(sleepTime < 0){
                sleepTime = 0;
            }
            try{
                sleep((long)(sleepTime*1000));//milliseconds
            }catch(Exception e){
                System.out.println(e);
                cameraActive = false;
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
    
    public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException{
        double rectLong, rectShort, aspectRatio, idealAspectRatio;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
	
        //Divide width by height to measure aspect ratio
        if(report.boundingRectWidth > report.boundingRectHeight){
            //particle is wider than it is tall, divide long by short
            aspectRatio = ratioToScore((rectLong/rectShort)/idealAspectRatio);
        } else {
            //particle is taller than it is wide, divide short by long
            aspectRatio = ratioToScore((rectShort/rectLong)/idealAspectRatio);
        }
	return aspectRatio;
    }
      
    private double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}
     
    private double scoreRectangularity(ParticleAnalysisReport report){
            if(report.boundingRectWidth*report.boundingRectHeight !=0){
                    return 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
            } else {
                    return 0;
            }
    }
    
    private boolean isHot(){
        return (areaPercent < desiredAreaPercent);
        
    }
    
    private boolean isWithinRange(){
        return (distance < maxDistance && distance > minDistance);
    }
    
    public boolean isReadyToShoot(){
        
        return (isHot() && isWithinRange());
    }
    
    public class NerdyCameraData extends Thread{
        public NerdyCameraData(){

        }
        
        public void run(){
            while(dataReportingActive){    
                System.out.println("Hello. I am Nerdy Camera Data.");
                SmartDashboard.putNumber("highestScoringParticle",highestScoreParticleNum);
                SmartDashboard.putNumber("top" , top );
                SmartDashboard.putNumber("left" , left );
                SmartDashboard.putNumber("height" , height ); 
                SmartDashboard.putNumber("width" , width ); 
                SmartDashboard.putNumber("area" , areaPercent);
                SmartDashboard.putNumber("The distance is : " , distance);
                SmartDashboard.putNumber("angle", angle);
                SmartDashboard.putBoolean("isWithinRange" , (distance < maxDistance && distance > minDistance));
                SmartDashboard.putBoolean("isHot" , (areaPercent < desiredAreaPercent));
            }
        }
        
    }
}