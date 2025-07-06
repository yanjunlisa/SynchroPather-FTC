package org.firstinspires.ftc.teamcode.synchropather;

public class DriveConstants {

    /**
     * GoBILDA 312 RPM motor has a 13.7:1 gearbox with a built-in encoder resolution
     * of 28 ticks per motor revolution
     */
    public static final double TICKS_PER_REV = 383.6;
    public static final double WHEEL_DIAMETER_INCHES=4.094;
    public static final double GEAR_RATIO=1.0;//direct drive
    public static final double TRACK_WIDTH=12.28;//the distance before left and right wheels
    //it is used to convert linear distance difference into angle in radians
    public static final double TICKS_TO_INCHES =
            (Math.PI * WHEEL_DIAMETER_INCHES)/(TICKS_PER_REV*GEAR_RATIO);
}
