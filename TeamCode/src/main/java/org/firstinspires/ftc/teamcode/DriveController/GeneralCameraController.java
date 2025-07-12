package org.firstinspires.ftc.teamcode.DriveController;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.summer.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.summer.SimpleColorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.List;

public class GeneralCameraController {
    public enum SampleColor {
        RED(),
        BLUE(),
        YELLOW()
    }
    private VisionPortal visionPortal;
    private VisionProcessor processor;
    private boolean processorEnabled = false;

    private final Size CAMERA_RESOLUTION = new Size(320, 240);
    private final Telemetry telemetry;
    public GeneralCameraController(WebcamName webcam, Telemetry telemetry){
        this.processor = new SimpleColorProcessor();
        this.visionPortal = buildVisionPortal(webcam);
        this.telemetry = telemetry;
    }
    public GeneralCameraController(VisionPortal visionPortal,VisionProcessor processor, Telemetry telemetry){
        this.visionPortal=visionPortal;
        this.processor = processor;
        this.telemetry = telemetry;
    }

    private VisionPortal buildVisionPortal(WebcamName cameraName) {
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(false)
                .addProcessors(this.processor)  // ADD PROCESSORS HERE
                .build();

        //visionPortal.setProcessorEnabled(processor, true);  // let processors run asynchronously using camera data
        //processorEnabled = true;

        return visionPortal;
    }
    /**
     * stop the camera
     */
    public void stopCamera(){
        visionPortal.close();
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

    public boolean getColorDetected(){
        List<Point> centers= getDetectedCenters();
        return (centers!=null && !centers.isEmpty());
    }

    public void periodic(){
        if (processorEnabled == true) {
            telemetry.addData("Camera", "Enabled");
        }else{
            telemetry.addData("Camera", "Disabled");
        }
    }
}
