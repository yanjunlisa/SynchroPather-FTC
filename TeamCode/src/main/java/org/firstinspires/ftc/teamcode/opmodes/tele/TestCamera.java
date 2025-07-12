package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveController.GeneralCameraController;
import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.summer.Robot;
import org.firstinspires.ftc.teamcode.summer.RobotLocalizationFourWheels;
import org.firstinspires.ftc.teamcode.summer.SimpleColorProcessor;

/**
 * This is the case showing how to test a color board in front of the robot
 * with an external camera
 * press the gamepad button
 */
@TeleOp(name="Example Robot's external camera")
public class TestCamera extends LinearOpMode {

    private RobotDriveController robotDriveController;
    private RobotLocalization robotLocalization;
    private GeneralCameraController cameraController;
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, false, this, telemetry);
        robotDriveController = robot.robotDriveController;
        robotLocalization = robot.robotLocalization;
        cameraController = robot.cameraController;

        waitForStart();
        robot.periodic();
        telemetry.update();

        boolean lastButtonState = false;

        while (opModeIsActive()) {
            boolean buttonPressed = this.gamepad1.a;

            // Detect rising edge: button was not pressed, now is pressed
            if (buttonPressed && !lastButtonState) {
                // Enable camera and set to detect RED
                cameraController.enableProcessor();
                cameraController.clearDetectedCenter();
                cameraController.setFilterColor(SimpleColorProcessor.SampleColor.RED);

                sleep(100);
                // Run detection
                boolean found = cameraController.getColorDetected();
                telemetry.addData("Detecting", "RED");
                telemetry.addData("RED Detected", found ? "YES" : "NO");
                telemetry.update();


                // Optionally, disable processor if you only want to detect on demand
                cameraController.disableProcessor();
                cameraController.clearDetectedCenter();
                found=false;
            }

            lastButtonState = buttonPressed;
          //  robot.periodic();
            sleep(50);
        }
        cameraController.disableProcessor();
        //robot.periodic();
        //telemetry.update();

    }

}

