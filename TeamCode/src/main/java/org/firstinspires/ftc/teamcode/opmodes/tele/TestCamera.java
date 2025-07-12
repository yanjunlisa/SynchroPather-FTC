package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveController.GeneralCameraController;
import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.summer.Robot;
import org.firstinspires.ftc.teamcode.summer.RobotLocalizationFourWheels;

/**
 * This is the case showing how to test a color board in front of the robot
 * with an external camera
 * press the gamepad button
 */
@TeleOp(name="Example Manual Robot")
public class TestCamera extends LinearOpMode {

    private RobotDriveController robotDriveController;
    private RobotLocalization robotLocalization;
    private GeneralCameraController cameraController;
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot= new Robot(hardwareMap,false,this, telemetry);

        robotDriveController = robot.robotDriveController;
        robotLocalization = robot.robotLocalization;
        cameraController = robot.cameraController;

        cameraController.setFilterColor(GeneralCameraController.SampleColor.RED);

        waitForStart();
        robot.periodic();
        telemetry.update();

        boolean lastButtonState = false;
        while(opModeIsActive()){
            //drive

            telemetry.addLine("Ready to detect Color Red, press button A.");
            telemetry.update();
            boolean buttonPressed = this.gamepad1.a;
            if (buttonPressed && !lastButtonState){
                cameraController.enableProcessor();
                cameraController.setFilterColor(GeneralCameraController.SampleColor.RED);
                telemetry.addLine("Color detection ENABLED");
            }else if(!buttonPressed && lastButtonState){
                cameraController.disableProcessor();
                telemetry.addLine("Color detection disabled");
            }
            lastButtonState=buttonPressed;

            if (buttonPressed){
                //only process color detection and show results when the button is held
                boolean found = cameraController.getColorDetected();
                if (found)
                    telemetry.addData("Color","found");
                else
                    telemetry.addData("Color","not found");
            }

            telemetry.update();

            robot.periodic();
            telemetry.update();
        };
        cameraController.disableProcessor();

        robot.periodic();
        telemetry.update();
    }

}

