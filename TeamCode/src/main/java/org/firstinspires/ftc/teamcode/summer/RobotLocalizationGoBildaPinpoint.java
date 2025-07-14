package org.firstinspires.ftc.teamcode.summer;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.summer.GoBildaPinpointDriver; // Adjust if your Pinpoint driver is elsewhere

/**
 * Uses GoBilda Pinpoint to find the location of the robot.
 */
public class RobotLocalizationGoBildaPinpoint extends RobotLocalization {
    private final GoBildaPinpointDriver pinpoint;

    public RobotLocalizationGoBildaPinpoint(GoBildaPinpointDriver pinpoint, Telemetry telemetry) {
        super(telemetry);
        this.pinpoint = pinpoint;
    }

    public void updatePosition() {
        Pose2d pose = pinpoint.getPosition(); // Assumes getPose() returns Pose2d
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getRotation().getRadians();
        update(x, y, heading);  // update fields in parent class
    }

    @Override
    public Pose2d getPose() {
        updatePosition();
        return new Pose2d(x, y, new Rotation2d(heading));
    }

    public void periodic() {
        telemetry.addData("Localization - X", x);
        telemetry.addData("Localization - Y", y);
        telemetry.addData("Localization - Heading", heading);
    }
}
