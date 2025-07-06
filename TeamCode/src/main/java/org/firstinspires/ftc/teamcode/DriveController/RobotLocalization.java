package org.firstinspires.ftc.teamcode.DriveController;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotLocalization {
    protected double x;
    protected double y;
    protected double heading;//in radians

    protected Telemetry telemetry;
    public RobotLocalization(Telemetry telemetry){
        this.x=0;
        this.y=0;
        this.heading=0;
        this.telemetry=telemetry;
    }

    public void update(double newX, double newY, double newHeading){
        this.x=newX;
        this.y=newY;
        this.heading=newHeading;
    }

    public double getHeading() {
        return heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Pose2d getPose(){
        return new Pose2d(x,y,new Rotation2d(heading));
    }

    public void periodic(){
        telemetry.addData("Localization - X",x);
        telemetry.addData("Localization - Y", y);
        telemetry.addData("Localization - Heading", heading);
    }
}
