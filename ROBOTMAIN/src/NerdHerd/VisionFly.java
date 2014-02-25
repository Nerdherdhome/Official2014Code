/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package NerdHerd;


import NerdHerd.Source.NerdyBot;
import com.sun.cldc.jna.Pointer;
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
public class VisionFly{
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    AxisCamera camera = AxisCamera.getInstance();
    CriteriaCollection cc;
    final int AREA_MINIMUM = 150;
    final int RECTANGULARITY_LIMIT = 50;
    final int ASPECT_RATIO_LIMIT = 65;
    final int boundingRectHeight = 11;
    final int imageHeight = 352;
    final int imageWidth = 240;
    final int MAX_PARTICLES = 8;
    public int processingStep = 0; 
    private int cameraProcessingCounter=0;
    private final int cameraProcessingCounterMax = (int)(2/NerdyBot.k_PeriodTime);//Approximate time for camera to update
    //target report analysis
    private TargetReport target;
    private int[] verticalTargets;
    private int[] horizontalTargets;
    private int horizontalTargetCount, verticalTargetCount;
    //classifying names
    private ColorImage image = null;
    private BinaryImage thresholdImage = null;
    private BinaryImage filteredImage = null;
    private BinaryImage objects = null;
    private NIVision.Rect boundingBox;
    //BinaryImage smallObjects = null;
    private Pointer imagePointer;
        
    public VisionFly() {
    cc = new CriteriaCollection();
    cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
    boundingBox = new NIVision.Rect(1, 1, 1, 1);
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

    public void cameraProcess() {
       

        //particle filtering
        int line = 0;
        //Determines when to run
        if(cameraProcessingCounter >= cameraProcessingCounterMax){
            processingStep = 1;
            cameraProcessingCounter = 0;
        }
        cameraProcessingCounter++;
        //Processing is broken into steps to alleviate use of the CPU
        try{
            if(processingStep == 1){
                //target report analysis
                target = new TargetReport();
                verticalTargets = new int[MAX_PARTICLES];
                horizontalTargets = new int[MAX_PARTICLES];

                image = camera.getImage();
                

                
                line ++;
                processingStep++;
            }else if (processingStep == 2){
                thresholdImage = image.thresholdHSV(93, 93, 77, 39, 100, 100); //green filter
                //thresholdImage = image.thresholdHSV(18, 27, 40, 255, 90, 255); //orange filter
                line ++;
                thresholdImage.write("/camera.bmp");
                line ++;
                processingStep++;
            }else if (processingStep == 3){
                objects = thresholdImage.removeLargeObjects(true, AREA_MINIMUM);
                line ++;
                objects.write("/lose.bmp");
                line ++;
                processingStep++;
            }else if (processingStep == 4){
                //smallObjects = objects.removeSmallObjects(true, AREA_MINIMUM);
                //line ++;
                //smallObjects.write("/second.bmp");
                //line ++;
                filteredImage = objects.particleFilter(cc);
                line ++;
                filteredImage.write("/test.bmp");     //test if it is filtering objects
                line ++;
                processingStep++;
            }else if (processingStep == 5){
                ParticleAnalysisReport [] reports = filteredImage.getOrderedParticleAnalysisReports();
                Scores scores[] = new Scores[filteredImage.getNumberParticles()];
                horizontalTargetCount = verticalTargetCount = 0;

                if(filteredImage.getNumberParticles() > 0){
                
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
                    try{
                        imagePointer = NIVision.imaqCreateImage(NIVision.ImageType.imaqImageRGB, 0);
                        NIVision.readFile(imagePointer, "/test.bmp");
                    }catch(NIVisionException e){
                        System.out.println(e);
                    }
                            
                    int top = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_TOP);
                    int left = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_LEFT);
                    int height = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT);
                    int width = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
                    
                    boundingBox.set(top, left, height, width);
                    
                    System.out.println(highestScoreParticleNum);
                }
                processingStep++;
            }else if (processingStep == 6){
                 processingStep = 0;
                 try{
                    thresholdImage.free();
                    objects.free();
                    //smallObjects.free();
                    filteredImage.free();
                    image.free();
                }catch (Exception ex) {
                    System.out.println(ex);
                }
            }
        } catch (Exception e) {
            System.out.println("Got To Line" + line);
            System.out.println(e);
            processingStep = 0;
             try{
                thresholdImage.free();
                objects.free();
                //smallObjects.free();
                filteredImage.free();
                image.free();
            }catch (Exception ex) {
                System.out.println(ex);
            }
        }
        
    
    }

    
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
     
public double scoreRectangularity(ParticleAnalysisReport report){
            if(report.boundingRectWidth*report.boundingRectHeight !=0){
                    return 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
            } else {
                    return 0;
            }
    }
    
public boolean getHighestScore(Scores scores, Scores compareScore){
        double sumScore = (scores.aspectRatioVertical + scores.aspectRatioHorizontal + scores.rectangularity);
        double sumCompareScore = (compareScore.aspectRatioVertical + compareScore.aspectRatioHorizontal + compareScore.rectangularity);
        return sumScore >= sumCompareScore;
        
    }
}