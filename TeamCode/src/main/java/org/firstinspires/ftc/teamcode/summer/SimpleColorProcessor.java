package org.firstinspires.ftc.teamcode.summer;

import android.graphics.Canvas;
import android.provider.ContactsContract;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.DriveController.GeneralCameraController;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class SimpleColorProcessor implements VisionProcessor {
    public static Scalar lowerWhite = new Scalar(0,0,200);
    public static Scalar upperWhite = new Scalar(180,30,255);
    public static Scalar lowerYellow = new Scalar(15.0, 100.0, 160.0); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 80.0, 100.0); // hsv
    public static Scalar upperBlue = new Scalar(140.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(160.0, 255.0, 255.0); // hsv
    public enum SampleColor {
        RED(),
        BLUE(),
        YELLOW(),
        WHITE()
    }
    private Mat frame;
    private List<Point> detectedCenter = new ArrayList<>();
    public static volatile SampleColor colorType = SampleColor.RED;

    public SimpleColorProcessor(){

    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public synchronized void setFilterColor(SampleColor color) {
        colorType = color;
    }

    public SampleColor getFilterColor(){
        return colorType;
    }

    /**
     * This function is called through the visionPortal object automatically, if
     * this SimpleColorProcessor object is attached to the visionPortal object, and
     * the processor is enabled, and
     * the camera stream run while the op mode is active.
     *
     * You do not call this function manually.
     * @param input
     * @param captureTimeNanos
     * @return
     */
    @Override
    public Object processFrame(Mat input, long captureTimeNanos){
        //convert input to HSV
        frame = input.clone();
        detectedCenter.clear();

        // Getting representative brightness of image
        Mat gray = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        //List of color ranges and colors form drawing
        Scalar lower=null;
        Scalar upper = null;
        Scalar drawColor=null;
        switch(colorType){
            case RED:
                lower=lowerRedH;
                upper=upperRedH;
                drawColor=new Scalar(0,0,255);
                break;
            case BLUE:
                lower=lowerBlue;
                upper=upperBlue;
                drawColor=new Scalar(255,0,0);
                break;
            case YELLOW:
                lower=lowerYellow;
                upper=upperYellow;
                drawColor=new Scalar(0,255,255);
                break;
            case WHITE:
                lower=lowerWhite;
                upper=upperWhite;
                drawColor=new Scalar(255,255,255);
                break;
        }

        Mat mask = new Mat();
        Core.inRange(gray,lower,upper,mask);

            //Find contours
        List<MatOfPoint> contours= new ArrayList<>();
        Imgproc.findContours(mask,contours,new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);

        //Draw contours
        for (MatOfPoint contour: contours){
            Imgproc.drawContours(input,List.of(contour),-1,drawColor,2);
            Moments moments=Imgproc.moments(contour);
            if(moments.get_m00()!=0) {
                double cx = moments.get_m10() / moments.get_m00();
                double cy = moments.get_m01() / moments.get_m00();
                Point center = new Point(cx, cy);
                detectedCenter.add(center);
                        //draw the center
                Imgproc.circle(input,center,4,drawColor,-1);
            }
        }
        return input;//return the image with contours drawn
    }

    public List<Point> getDetectedCenter(){
        return detectedCenter;
    }

    public void clearDetectedCenter(){
        detectedCenter = new ArrayList<>();
    }
}
