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

/**
 * This is the case showing how to manually run the robot with a gamepad.
 * Flag manual= true
 * The robot is initialized with a drive controller and a localizer.
 * gamepad.left_stick_y: push stick forward gives negative value, so the value is reversed in the code.
 * gamepad.left_stick_x: show the side way motion - right or left
 * gamepad.right_stick_x: show the rotation, clockwise and counter clockwise
 * The drive controller drives the robot by raw power.
 * use the driveRobotCentric function of controller class to move the robot relative to itself.
 */
@TeleOp(name="Example Manual Robot")
public class TestTeleOp extends LinearOpMode{

    private RobotDriveController robotDriveController;
    private RobotLocalization robotLocalization;
    private Robot robot;

    private double applyDeadzone(double value, double threshold){
        return Math.abs(value)<threshold ? 0: value;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //set the robot with raw power controlled motors
        boolean manul=true;
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
            double turn = applyDeadzone(gamepad1.right_stick_x,0.05);//left motor spin forward, right motor spin backward
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.update();
            //control the robot with raw inputs from the gamepad.
            robotDriveController.controller_driveRobotCentric(forward,strafe, turn);
            /**
             * The following part is to use imu and drive the robot relative to the field.
             */
            //robotDriveController.controller_driveFieldCentric(forward,strafe,turn);

            //double speed = TranslationConstants.MAX_VELOCITY;
            //robot.robotDriveController.driveFieldCentric(strafe * speed,forward * speed,turn * speed);
            //robotDriveController.driveRobotCentricPowers(strafe * speed,forward * speed,turn * speed);
            ((RobotLocalizationFourWheels)robotLocalization).updatePosition();
            robot.periodic();
            telemetry.update();
        };
        robot.periodic();
        telemetry.update();
    }

}
