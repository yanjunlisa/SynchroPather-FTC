package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.summer.Robot;
import org.firstinspires.ftc.teamcode.synchropather.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.translation.movements.LinearTranslation;

@Autonomous(name="Forward PIDF Tuner", group = "Calibration")
public class ForwardPIDFTuner extends LinearOpMode {

    Synchronizer synchronizer;

    //TODO: Make sure these are fully coded
    private RobotDriveController robotDriveController;
    private RobotLocalization robotLocalization;
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        initSubsystems();
        initSynchronizer();

        waitForStart();

        while (opModeIsActive()) {


            while (opModeIsActive() && !gamepad1.square) {
                if (synchronizer.getIsRunning()) {
                    synchronizer.update();
                }
            }

            initSynchronizer();
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), robotLocalization.getPose());
                TranslationState targetTranslation = (TranslationState) synchronizer.getState(MovementType.TRANSLATION);
                Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(targetTranslation.getX(), targetTranslation.getY(), new Rotation2d()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean manul=false;
        robot= new Robot(hardwareMap,manul,this, telemetry);
        //TODO: Init robotDriveController here
        robotDriveController = robot.robotDriveController;

        //TODO: Init robotLocalization here
        // Make sure your robot's starting position is (0,0) ?
        robotLocalization = robot.robotLocalization;

    }


    private void initSynchronizer() {

        // Translation plan
        LinearTranslation line1 = new LinearTranslation(0,
                new TranslationState(0, 0),
                new TranslationState(48, 0)
        );
        LinearTranslation line2 = new LinearTranslation(line1.getEndTime()+1,
                new TranslationState(48, 0),
                new TranslationState(0, 0)
        );
        TranslationPlan translationPlan = new TranslationPlan(
                robotDriveController,robotLocalization,
                line1,line2
        );

        this.synchronizer = new Synchronizer(
               telemetry,translationPlan
        );

    }

}