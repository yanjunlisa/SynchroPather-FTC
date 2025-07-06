package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.summer.Robot;
import org.firstinspires.ftc.teamcode.summer.RobotLocalizationFourWheels;
import org.firstinspires.ftc.teamcode.synchropather.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.translation.movements.CRSplineTranslation;

@TeleOp(name="Example SynchroPather Manual")
public class TestTeleOp extends LinearOpMode {

    private RobotDriveController robotDriveController;
    private RobotLocalization robotLocalization;
    private Robot robot;

    private double applyDeadzone(double value, double threshold){
        return Math.abs(value)<threshold ? 0: value;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        boolean manul=false;
        robot= new Robot(hardwareMap,manul,this, telemetry);
        robotDriveController = robot.robotDriveController;
        robotLocalization = robot.robotLocalization;

        waitForStart();
        robot.periodic();
        telemetry.update();

        while(opModeIsActive()){
            //drive
            double forward = -applyDeadzone(gamepad1.left_stick_y,0.05);
            double strafe = applyDeadzone(gamepad1.left_stick_x,0.05);
            double turn = applyDeadzone(gamepad1.right_stick_x,0.05);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.update();
            if (Math.abs(turn) > 0) {
                robot.robotDriveController.changeInversions();
            }
            double speed = TranslationConstants.MAX_VELOCITY;
            //robot.robotDriveController.driveFieldCentric(strafe * speed,forward * speed,turn * speed);
            robotDriveController.driveRobotCentricPowers(strafe * speed,forward * speed,turn * speed);
            ((RobotLocalizationFourWheels)robotLocalization).updatePosition();
            robot.periodic();
            telemetry.update();
        };
        robot.periodic();
        telemetry.update();
    }

}
