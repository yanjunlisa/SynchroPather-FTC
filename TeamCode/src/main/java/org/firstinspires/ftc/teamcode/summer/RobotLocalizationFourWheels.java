package org.firstinspires.ftc.teamcode.summer;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;

/**
 * use encoder of four wheels to find the location of the robot
 */
public class RobotLocalizationFourWheels extends RobotLocalization {
    private MotorEx frontLeft, frontRight, backLeft, backRight;
    private int lastFL, lastFR, lastBL, lastBR;

    public RobotLocalizationFourWheels(MotorEx fL,
                                       MotorEx fR,
                                       MotorEx bL,
                                       MotorEx bR,
                                       Telemetry telemetry){
        super(telemetry);
        this.frontLeft = fL;
        this.frontLeft.resetEncoder();
        this.frontRight = fR;
        this.frontRight.resetEncoder();
        this.backLeft= bL;
        this.backLeft.resetEncoder();
        this.backRight = bR;
        this.backRight.resetEncoder();

        lastFL=0;
        lastFR=0;
        lastBL=0;
        lastBR=0;

        //telemetry.addData("Robot Localizer", "Initialized");
        //telemetry.update();
    }

    public void updatePosition(){
        int currentFL = frontLeft.getCurrentPosition();
        int currentFR = frontRight.getCurrentPosition();
        int currentBL = backLeft.getCurrentPosition();
        int currentBR = backRight.getCurrentPosition();

        int deltaFL = currentFL - lastFL;
        int deltaFR = currentFR - lastFR;
        int deltaBL = currentBL - lastBL;
        int deltaBR = currentBR - lastBR;

        lastFL = currentFL;
        lastFR = currentFR;
        lastBL = currentBL;
        lastBR = currentBR;
        double dx = (deltaFL+deltaFR+deltaBL+deltaBR)/4.0* DriveConstants.TICKS_TO_INCHES;
        double dy =(-deltaFL+deltaFR+deltaBL-deltaBR)/4.0*DriveConstants.TICKS_TO_INCHES;
        double dTheta = (-deltaFL+deltaFR-deltaBL+deltaBR)/(4*DriveConstants.TRACK_WIDTH);
        update(getX()+dx,getY()+dy,getHeading()+dTheta);
    }

    @Override
    public Pose2d getPose(){
        updatePosition();
        return new Pose2d(x,y,new Rotation2d(heading));
    }

    public void periodic(){
        telemetry.addData("Localization - X",x);
        telemetry.addData("Localization - Y", y);
        telemetry.addData("Localization - Heading", heading);
    }
}
