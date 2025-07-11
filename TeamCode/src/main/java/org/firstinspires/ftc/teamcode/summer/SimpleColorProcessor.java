package org.firstinspires.ftc.teamcode.summer;

import android.graphics.Canvas;
import android.provider.ContactsContract;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SimpleColorProcessor implements VisionProcessor {
    public static Scalar lowerYellow = new Scalar(15.0, 100.0, 160.0); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 80.0, 100.0); // hsv
    public static Scalar upperBlue = new Scalar(140.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(160.0, 255.0, 255.0); // hsv

    private Mat frame;


    public SimpleColorProcessor(){

    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    @Override
    public Object processFrame(Mat input, long captureTimeNanos){
        //convert input to HSV
        frame = input.clone();

        // Getting representative brightness of image
        Mat gray = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        //List of color ranges and colors form drawing
        Scalar[] lowers={lowerRedH,lowerBlue,lowerYellow};
        Scalar[] uppers={upperRedH, upperBlue, upperYellow};
        Scalar[] drawColors={new Scalar(0,0,255),//Red (BGR)
                            new Scalar(255,0,0),//Blue (BGR)
                            new Scalar(0,255,255)};   //Yellow (BGR)
        for (int i=0; i< lowers.length; i++){
            Mat mask = new Mat();
            Core.inRange(gray,lowers[i],uppers[i],mask);

            //Find contours
            List<MatOfPoint> contours= new ArrayList<>();
            Imgproc.findContours(mask,contours,new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);

            //Draw contours
            Imgproc.drawContours(input,contours,-1,drawColors[i],2);
        }
        return input;//return the image with contours drawn
    }
}
