package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.summer.Robot;
import org.firstinspires.ftc.teamcode.synchropather.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationState;

@Autonomous(name="Example SynchroPather Auto")
public class ExampleSynchroPatherAuto extends LinearOpMode {

    Synchronizer synchronizer;
    private RobotDriveController robotDriveController;
    private RobotLocalization robotLocalization;
    private Robot robot;
    //private final Telemetry telemetry;

    private double applyDeadzone(double value, double threshold){
        return Math.abs(value)<threshold ? 0: value;
    }
    @Override
    public void runOpMode() throws InterruptedException {

        //this.robot= new Robot(hardwareMap,manul,this, telemetry);

        initSubsystems();
        initSynchronizer();

        waitForStart();
        synchronizer.periodic();
        robot.periodic();
        telemetry.update();

        synchronizer.start();
        while (opModeIsActive() && synchronizer.update()){
            if (synchronizer.getIsRunning()){
                telemetry.addData("Synchronizer","Running");
                telemetry.update();
            }
        }
        synchronizer.stop();


    }


    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.robot= new Robot(hardwareMap,false,this, telemetry);

        //TODO: Init robotDriveController here
        this.robotDriveController = robot.robotDriveController;

        //TODO: Init robotLocalization here
        // Make sure your robot's starting position is (0,0) ?
        this.robotLocalization = robot.robotLocalization;

    }
    private void initSynchronizer() {
        // Translation plan
        CRSplineTranslation spline = new CRSplineTranslation(0,
                new TranslationState(0, 24),
                new TranslationState(24, 0),
                new TranslationState(0, -24),
                new TranslationState(-24, 0),
                new TranslationState(0, 24)
        );
        TranslationPlan translationPlan = new TranslationPlan(robotDriveController,
                robotLocalization,spline);

        // Rotation plan
        LinearRotation rotation = new LinearRotation(new TimeSpan(0, spline.getEndTime()),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(270))
        );

        RotationPlan rotationPlan = new RotationPlan(robotDriveController,robotLocalization,rotation);

        // Synchronizer
        this.synchronizer = new Synchronizer(
                telemetry,translationPlan,
                rotationPlan
        );
    }

}
