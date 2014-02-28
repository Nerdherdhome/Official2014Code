/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package NerdHerd;


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
public class Camera{
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
    private Pointer imagePointer;
    public int processingStep = 0; 
    private int cameraProcessingCounter=0;
    private final int cameraProcessingCounterMax = (int)(2/NerdyBot.k_PeriodTime);//Approximate time for camera to update
    private ColorImage image = null;
    private BinaryImage thresholdImage = null;
    private BinaryImage filteredImage = null;
    private BinaryImage objects = null;
    private Scores scores[] = new Scores[8];
    private int highestScoreParticleNum = 0;
    int top = 0;
    int left = 0;
    int height = 0;
    int width = 0;
    double areaPercent = 0;
    double distance = 0;

     
    public void Init(AxisCamera.ResolutionT resolution) {
        camera.writeResolution(resolution);
        System.out.println(camera.getResolution());
        cc = new CriteriaCollection();
        cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        rr = new CriteriaCollection();
        rr.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 60, 160,false);
    }
    
    public class Scores{
        double rectangularity;
        double aspectRatioHorizontal;
        double aspectRatioVertical;
    }
    
    public void cameraProcess() {
        //classifying names
        image = null;
        thresholdImage = null;
        filteredImage = null;
        objects = null;
        //BinaryImage smallObjects = null;
        //target report analysis

        //particle filtering

        if(cameraProcessingCounter >= cameraProcessingCounterMax){
            processingStep = 1;
            cameraProcessingCounter = 0;
        }
        try{
            if(processingStep == 1){
                image = camera.getImage();
            }else if (processingStep == 2){
                //thresholdImage = image.thresholdHSV(135, 170, 85, 100, 100, 100); //green filter
                //thresholdImage = image.thresholdHSV(18, 27, 40, 255, 90, 255); //orange filter
                thresholdImage = image.thresholdRGB(150, 255, 150, 255, 50, 150); //orange filter
            }else if (processingStep == 3){
                thresholdImage.write("/camera.bmp");
            }else if (processingStep == 4){
                objects = thresholdImage.removeLargeObjects(true, AREA_MINIMUM);
            }else if (processingStep == 5){
                objects.write("/lose.bmp");
            }else if (processingStep == 6){
                filteredImage = objects.particleFilter(cc);
            }else if (processingStep == 7){
                filteredImage.write("/test.bmp");     //test if it is filtering objects
            }else if (processingStep == 8){
                ParticleAnalysisReport [] reports = filteredImage.getOrderedParticleAnalysisReports();
            }else if (processingStep == 9){
                Scores scores[] = new Scores[filteredImage.getNumberParticles()];

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
            }else if (processingStep == 10){
                boolean highestScore = false;
                highestScoreParticleNum = 0;
                for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles();i++) {

                    highestScore = getHighestScore(scores[i], scores[highestScoreParticleNum]);

                    if(highestScore){
                        highestScoreParticleNum = i;
                    }
                }
            }else if (processingStep == 11){
                imagePointer = filteredImage.image;
            }else if (processingStep == 12){
                NIVision.particleFilter(imagePointer, imagePointer, cc);
                int particles = NIVision.countParticles(imagePointer);
                highestScoreParticleNum = 0;
                for(int i = 0; i < particles; i++){

                    int area1 = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
                    int area2 = (int)NIVision.MeasureParticle(imagePointer, i, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);

                    if(area2 > area1){
                        highestScoreParticleNum = i;
                    }
                }
            }else if (processingStep == 13){

                top = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_TOP);
                left = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_LEFT);
                height = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT);
                width = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH);
                areaPercent = (int)NIVision.MeasureParticle(imagePointer, highestScoreParticleNum, false, MeasurementType.IMAQ_MT_AREA);
                areaPercent /= width*height;
                distance = 2*imageWidth/(2*width*Math.tan(0.4101523));
            }else if (processingStep == 13){
                System.out.println(highestScoreParticleNum);
                System.out.println("top" + top + "left" + left + "height" + height + "width" + width + "area" + areaPercent);
                System.out.println("The distance is : " + distance);
            }
        } catch (Exception e) {
            System.out.println(e);
        }finally{
            try{
                thresholdImage.free();
                objects.free();
                //smallObjects.free();
                filteredImage.free();
                image.free();
                //NIVision.dispose(imagePointer);
                imagePointer.free();
            }catch (Exception ex) {
            }
        }
    }
    
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
