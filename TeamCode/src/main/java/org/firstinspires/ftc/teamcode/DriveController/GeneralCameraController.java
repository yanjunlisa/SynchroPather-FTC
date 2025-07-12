package org.firstinspires.ftc.teamcode.DriveController;

import org.firstinspires.ftc.teamcode.summer.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.summer.SimpleColorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.List;

public class GeneralCameraController {
    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED()
    }
    private VisionPortal visionPortal;
    private VisionProcessor processor;
    private boolean processorEnabled = false;

    public GeneralCameraController(VisionPortal visionPortal,VisionProcessor processor){
        this.visionPortal=visionPortal;
        this.processor = processor;
    }

    public void enableProcessor(){
        visionPortal.setProcessorEnabled(processor,true);
        processorEnabled=true;
    }

    public void disableProcessor(){
        visionPortal.setProcessorEnabled(processor,false);
        processorEnabled = false;
    }

    public void setFilterColor(SampleColor c){
        if (processor instanceof SampleOrientationProcessor){
            ((SampleOrientationProcessor) processor).setFilterColor(c);
        }else if (processor instanceof SimpleColorProcessor){
            ((SimpleColorProcessor)processor).setFilterColor(c);
        }
    }

    public List<double[]> getSamplePositions(){
        if (processor instanceof SampleOrientationProcessor){
            return ((SampleOrientationProcessor)processor).getRealPositions();
        }

        return new ArrayList<>();
    }

    public List<Point> getDetectedCenters(){
        if (processor instanceof SimpleColorProcessor){
            return ((SimpleColorProcessor)processor).getDetectedCenter();
        }
        return new ArrayList<>();
    }

}
