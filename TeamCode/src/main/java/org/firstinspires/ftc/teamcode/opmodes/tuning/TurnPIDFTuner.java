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
import org.firstinspires.ftc.teamcode.synchropather.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.rotation.movements.LinearRotation;

@Autonomous(name="Turn PIDF Tuner", group = "Calibration")
public class TurnPIDFTuner extends LinearOpMode {

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
                RotationState targetRotation = (RotationState) synchronizer.getState(MovementType.ROTATION);
                Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(0, 0, new Rotation2d(targetRotation.getHeading())));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        boolean manul=false;
        this.robot= new Robot(hardwareMap,manul,this, telemetry);
        //TODO: Init robotDriveController here
        this.robotDriveController = robot.robotDriveController;

        //TODO: Init robotLocalization here
        // Make sure your robot's start heading is 0 radians?
        this.robotLocalization = robot.robotLocalization;

    }


    private void initSynchronizer() {

        // Rotation plan
        LinearRotation rot1 = new LinearRotation(0,
                new RotationState(0),
                new RotationState(Math.toRadians(180))
        );
        LinearRotation rot2 = new LinearRotation(rot1.getEndTime()+1,
                new RotationState(Math.toRadians(180)),
                new RotationState(0)
        );

        RotationPlan rotationPlan = new RotationPlan(robotDriveController, robotLocalization,
                rot1,
                rot2
        );

        this.synchronizer = new Synchronizer(
              telemetry,  rotationPlan
        );

    }

}