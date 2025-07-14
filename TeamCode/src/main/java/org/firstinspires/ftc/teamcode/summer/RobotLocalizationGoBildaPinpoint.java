package org.firstinspires.ftc.teamcode.summer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;

import java.util.ArrayList;

public class RobotLocalizationGoBildaPinpoint extends RobotLocalization {
    // Kalman Filter variables and history arrays
    //uncertainty
    private double P_translation, P_heading;
    private static final double Q_translation=0.01849838438;
    private static final double Q_heading=0.0001;
    //Position and velocity
    private double x, y, h, vx, vy, vh;
    private final ArrayList<Double> xHistory, yHistory, hHistory, dtHistory;
    private double lastTime, deltaTime, averageDeltaTime;
    private final ElapsedTime runtime;
    private GoBildaPinpointDriver pinpoint;
    private Pose2d lastOdometryPose;
    private Telemetry telemetry;

    private static final double xOffset=92;
    private static final double yOffset=48;
    private static final double ENCODER_CPR = 4096;//optii v1
    private static final double ODOM_DIAMETER =35.0;//mm
    private static final double TICKS_PER_MM=ENCODER_CPR/(Math.PI * ODOM_DIAMETER);

    public RobotLocalizationGoBildaPinpoint(Pose2d initialPose,
                                            GoBildaPinpointDriver pinpoint,
                                            LinearOpMode opMode, Telemetry telemetry) {
        super(telemetry);
        this.telemetry = telemetry;
        this.P_translation = 2;
        this.P_heading = 0.1;

        this.x = initialPose.getX();
        this.y = initialPose.getY();
        this.h = initialPose.getHeading();

        this.xHistory = new ArrayList<>();
        xHistory.add(x);
        this.yHistory = new ArrayList<>();
        yHistory.add(y);
        this.hHistory = new ArrayList<>();
        hHistory.add(h);

        this.vx = 0;
        this.vy = 0;
        this.vh = 0;

        this.runtime = new ElapsedTime();
        this.runtime.reset();
        this.lastTime = -1;
        this.deltaTime = 1;
        this.dtHistory = new ArrayList<>();
        dtHistory.add(1d);
        this.averageDeltaTime = 1;

        //init pinpoint
        this.pinpoint = pinpoint;
        this.pinpoint.setOffsets(xOffset, yOffset);
        this.pinpoint.setEncoderResolution(TICKS_PER_MM);
        // Initialize pinpoint and set initial pose (see lines 132-174 of LocalizationSubsystem)
        // ...
        initializePinpoint(opMode,initialPose);
        this.lastOdometryPose = initialPose;
    }

    public void initializePinpoint(LinearOpMode opMode,Pose2d initialPose){
        // wait for pinpoint to be ready
        this.telemetry.addData("[Pinpoint LOCALIZATION]", "initializing pinpoint");
        this.telemetry.update();
        while ((opMode.opModeIsActive() || opMode.opModeInInit())
                && this.pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            this.pinpoint.update();
            this.telemetry.addData("[PP STATUS]", this.pinpoint.getDeviceStatus());
            this.telemetry.update();
        }
        this.telemetry.addData("[PP STATUS]", this.pinpoint.getDeviceStatus());
        this.telemetry.update();
        this.pinpoint.update();

        // set pinpoint initial position
        this.telemetry.addData("[LOCALIZATION]", "setting pinpoint initial position");
        this.telemetry.update();
        Pose2D initialPose2D = new Pose2D(DistanceUnit.INCH,
                initialPose.getX(), initialPose.getY(),
                AngleUnit.RADIANS, initialPose.getHeading());
        double initialX = initialPose.getX();
        double initialY = initialPose.getY();
        double initialH = initialPose.getHeading();
        Pose2D ppPose = this.pinpoint.getPosition();
        double ppX = ppPose.getX(DistanceUnit.INCH);
        double ppY = ppPose.getY(DistanceUnit.INCH);
        double ppH = ppPose.getHeading(AngleUnit.RADIANS);
        while ((opMode.opModeIsActive() || opMode.opModeInInit())
                && !(equal(ppX,initialX) && equal(ppY,initialY) && equal(ppH,initialH))) {
            this.pinpoint.setPosition(initialPose2D);
            this.pinpoint.update();
            ppPose = this.pinpoint.getPosition();
            ppX = ppPose.getX(DistanceUnit.INCH);
            ppY = ppPose.getY(DistanceUnit.INCH);
            ppH = ppPose.getHeading(AngleUnit.RADIANS);

            // Telemetry
            this.telemetry.addData("[PP LOCALIZATION] ppX", ppX);
            this.telemetry.addData("[PP LOCALIZATION] ppY", ppY);
            this.telemetry.addData("[PP LOCALIZATION] ppH", ppH);
            this.telemetry.addData("[PP LOCALIZATION] initialH", initialH);
            this.telemetry.update();
        }
        this.telemetry.addData("[L. SUB STATUS]", "finished initializing pinpoint");
        this.telemetry.update();
    }


    public void update() {
        predictKF();
        updateVelocity();
        // Telemetry as shown in LocalizationSubsystem
        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("H", Math.toDegrees(h)+"°");
            telemetry.addData("VX", vx+"in/s");
            telemetry.addData("VY", vy+"in/s");
            telemetry.addData("VH", Math.toDegrees(vh)+"°/s");
            telemetry.update();
        }
    }

    private void predictKF() {
        pinpoint.update();
        Pose2D pinpointPose = pinpoint.getPosition();
        Pose2d currentPose = new Pose2d(pinpointPose.getX(DistanceUnit.INCH), pinpointPose.getY(DistanceUnit.INCH), new Rotation2d(pinpointPose.getHeading(AngleUnit.RADIANS)));
        double odom_dx = currentPose.getX() - lastOdometryPose.getX();
        double odom_dy = currentPose.getY() - lastOdometryPose.getY();
        double beta = normalizeAngle(h - lastOdometryPose.getHeading());
        double dx = odom_dx * Math.cos(beta) - odom_dy * Math.sin(beta);
        double dy = odom_dx * Math.sin(beta) + odom_dy * Math.cos(beta);
        x += dx; y += dy;
        double odom_dh = normalizeAngle(currentPose.getHeading() - lastOdometryPose.getHeading());
        h = normalizeAngle(h + odom_dh);
        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(h)) {
            x = lastOdometryPose.getX();
            y = lastOdometryPose.getY();
            h = lastOdometryPose.getHeading();
        } else {
            lastOdometryPose = currentPose;
        }
        P_translation += Math.hypot(dx, dy) * Q_translation;
        P_heading += Math.abs(odom_dh) * Q_heading;

        // Telemetry
        if (telemetry!=null) {
            telemetry.addData("Odom X", x);
            telemetry.addData("Odom Y", y);
            telemetry.addData("Odom H", Math.toDegrees(h)+"°");
            telemetry.addData("P_translation", P_translation);
            telemetry.addData("P_heading", P_heading);
            telemetry.addData("PP X", pinpointPose.getX(DistanceUnit.INCH));
            telemetry.addData("PP Y", pinpointPose.getY(DistanceUnit.INCH));
            telemetry.addData("PP H", pinpointPose.getHeading(AngleUnit.DEGREES));
        }
    }

    private void updateVelocity() {
        double currentTime = runtime.seconds();
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        dtHistory.add(deltaTime); xHistory.add(x); yHistory.add(y); hHistory.add(h);
        while (dtHistory.size() > 5) dtHistory.remove(0);
        while (xHistory.size() > 5) xHistory.remove(0);
        while (yHistory.size() > 5) yHistory.remove(0);
        while (hHistory.size() > 5) hHistory.remove(0);
        averageDeltaTime = dtHistory.stream().mapToDouble(aa -> aa).average().orElse(0);
        if (xHistory.size() == 5) vx = stencil(xHistory);
        if (yHistory.size() == 5) vy = stencil(yHistory);
        if (hHistory.size() == 5) vh = stencil(hHistory);
    }

    private double stencil(ArrayList<Double> a) {
        return (-a.get(4) + 8*a.get(3) - 8*a.get(1) + a.get(0)) / (12 * averageDeltaTime);
    }

    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private static boolean equal(double a, double b) {
        return Math.abs(a - b) <= 1e-3;
    }

    // Add pose/velocity getters as in LocalizationSubsystem
}