/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import NerdHerd.Source.NerdyBot;
import com.sun.cldc.jna.Pointer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.camera.AxisCamera;
//import static edu.wpi.first.wpilibj.communication.UsageReporting.report;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.NIVision.Rect;

//import edu.wpi.first.wpilibj.templates.RobotTemplate.scores;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class DummyCode extends NerdyBot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    AxisCamera camera = AxisCamera.getInstance();
    CriteriaCollection cc;
    CriteriaCollection rr;
    final int AREA_MINIMUM = 150;
    final int RECTANGULARITY_LIMIT = 50;
    final int ASPECT_RATIO_LIMIT = 65;
    final int boundingRectHeight = 11;
    final int imageHeight = 120;
    final int imageWidth = 160;
    final int MAX_PARTICLES = 8;
    private NIVision.Rect boundingBox = new NIVision.Rect(0, 0, 352, 240);
    private Pointer imagePointer;
    
     
    public void robotInit() {
    camera.writeResolution(AxisCamera.ResolutionT.k160x120);
    System.out.println(camera.getResolution());
    cc = new CriteriaCollection();
    cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
    rr = new CriteriaCollection();
    rr.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 60, 160,false);
    
    
    //cc.addCriteria(MeasurementType.)
    //cc.addCriteria(MeasurementType.IMAQ_MT_AREA, imageWidth, imageHeight, true);
    //cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_DIAGONAL, , true);
    //cc.addCriteria(MeasurementType.IMAQ_MT_PERIMETER, imageWidth, imageHeight);
    }
    
    public class Scores{
        double rectangularity;
        double aspectRatioHorizontal;
        double aspectRatioVertical;
    }
    
    public class TargetReport {
        
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    //classifying names
    ColorImage image = null;
    BinaryImage thresholdImage = null;
    BinaryImage filteredImage = null;
    BinaryImage objects = null;
    //BinaryImage smallObjects = null;
    BinaryImage refilteredImage = null;
    //target report analysis
    TargetReport target = new TargetReport();
    int verticalTargets[] = new int[MAX_PARTICLES];
    int horizontalTargets[] = new int[MAX_PARTICLES];
    int verticalTargetCount, horizontalTargetCount;
    
    //particle filtering
            int line = 0;

    try{
        image = camera.getImage();
        line ++;//1
        //thresholdImage = image.thresholdHSV(135, 170, 85, 100, 100, 100); //green filter
        //thresholdImage = image.thresholdHSV(18, 27, 40, 255, 90, 255); //orange filter
        thresholdImage = image.thresholdRGB(150, 255, 150, 255, 50, 150); //orange filter
        line ++;//2
        thresholdImage.write("/camera.bmp");
        line ++;//3
        objects = thresholdImage.removeLargeObjects(true, AREA_MINIMUM);
        line ++;//4
        objects.write("/lose.bmp");
        line ++;//5
        //smallObjects = objects.removeSmallObjects(true, AREA_MINIMUM);
        //line ++;
        //smallObjects.write("/second.bmp");
        //line ++;
        filteredImage = objects.particleFilter(cc);
        line ++;//6
        filteredImage.write("/test.bmp");     //test if it is filtering objects
        line ++;//7
        refilteredImage = filteredImage.particleFilter(cc);
        line ++;//8
        ParticleAnalysisReport [] reports = filteredImage.getOrderedParticleAnalysisReports();
        line ++;//9
        Scores scores[] = new Scores[filteredImage.getNumberParticles()];
        line ++;//10
        horizontalTargetCount = verticalTargetCount = 0;
        line ++;//11
        
        if(filteredImage.getNumberParticles() > 0)
        {
            for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles();i++) {
            ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
            scores[i] = new Scores();
            
            //score if it can score
           scores[i].rectangularity = scoreRectangularity(report);
	   scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
	   scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);
          
           
          
              
        }
            

            //check system to enage the goal 
            //scores[i].aspectRatioInner = scoresAspectRatio(filteredImage, reports, true);
            boolean highestScore = false;
            int highestScoreParticleNum = 0;
            for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles();i++) {
                
                highestScore = getHighestScore(scores[i], scores[highestScoreParticleNum]);

                if(highestScore){
                    highestScoreParticleNum = i;
                }
            }
            
            imagePointer = NIVision.imaqCreateImage(NIVision.ImageType.imaqImageHSL, 1);
            line ++;//12
            NIVision.readFile(imagePointer, "/camera.bmp");
            imagePointer = filteredImage.image;
            line ++;//13
            
            line ++;//14
           
            //NIVision.particleFilter(imagePointer, imagePointer, rr);
            NIVision.particleFilter(imagePointer, imagePointer, cc);
            
            
            line ++;//15
            int particles = NIVision.countParticles(imagePointer);
            
           
            highestScoreParticleNum = 0;
            for(int i = 0; i < particles; i++){
                
                int area1 = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
                int area2 = (int)NIVision.MeasureParticle(imagePointer, i, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
                
                if(area2 > area1){
                    highestScoreParticleNum = i;
                }
            }
            
            int top = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_TOP);
            line ++;
            int left = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_LEFT);
            line ++;
            int height = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT);
            line ++;
            int width = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
            line ++;
            double areaPercent = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_AREA);
            areaPercent /= width*height;
            double distance = 0;
            
            
            //distance = (width / Math.tan(80*Math.PI/180))*2/width*320/width;
            //distance = (((2/width)*320)/2)/(Math.tan(23.5/180*Math.PI));
            distance = 2*imageWidth/(2*width*Math.tan(0.4101523));
            
            System.out.println(highestScoreParticleNum);
            System.out.println("top" + top + "left" + left + "height" + height + "width" + width + "area" + areaPercent);
            System.out.println("The number of particles is : " + particles);
            System.out.println("The distance is : " + distance);
            
            System.out.println("Time Period\t" + m_mainPeriodicTimer.getLastActualPeriod());
            
        }
    
        
    } catch (Exception e) {
        System.out.println("Got To Line" + line);
        System.out.println(e);
    }finally{
        try{
            thresholdImage.free();
            objects.free();
            //smallObjects.free();
            filteredImage.free();
            image.free();
            boundingBox.free();
            //NIVision.dispose(imagePointer);
            imagePointer.free();
           
        }catch (Exception ex) {
            
        }
    }
    
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
     //boolean scoreCompare(Scores scores, boolean vertical){
	//boolean isTarget = true;

	//isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
	//if(vertical){
            //isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
	//} else {
            //isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
	//}

	//return isTarget;
    //}
    
    /**
     * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
     * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
     * 
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
      public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException
    {
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
      double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}
     
    double scoreRectangularity(ParticleAnalysisReport report){
            if(report.boundingRectWidth*report.boundingRectHeight !=0){
                    return 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
            } else {
                    return 0;
            }
    }
    
    boolean getHighestScore(Scores scores, Scores compareScore){
        double sumScore = (scores.aspectRatioVertical + scores.aspectRatioHorizontal + scores.rectangularity);
        double sumCompareScore = (compareScore.aspectRatioVertical + compareScore.aspectRatioHorizontal + compareScore.rectangularity);
        return sumScore >= sumCompareScore;
        
    }
}