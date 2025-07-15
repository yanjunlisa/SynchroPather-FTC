package org.firstinspires.ftc.teamcode.summer;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveController.GeneralCameraController;
import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.vision.VisionPortal;

public class Robot {
    /**
     * The opMode component of type LinearOpMode.
     */
    public final LinearOpMode opMode;
    /**
     * The drive component of type DriveSubsystem.
     */
    //public final DriveSubsystem drive;
    public final RobotDriveController robotDriveController;
    public final RobotLocalization robotLocalization;

    /**
     * The arm drive
     */
    //public final ArmSubsystem armDrive;

    /**
     * The drive for webcam
     */
    public final GeneralCameraController cameraController;
/**
 * The inDep component of type indepSubsystem.
 */
    // public final indepSubsystem inDep;
    /**
     * The hardwareRobot component of type HardwareRobot.
     */
    public final HardwareRobot hardwareRobot;

    public final Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, boolean manul, LinearOpMode opMode, Telemetry telemetry) {
        this.opMode = opMode;
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        this.robotDriveController = new RobotDriveController(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,//hardwareRobot.imu,
                manul,telemetry
        );
        //this.armDrive = new ArmSubsystem(hardwareRobot.elbow,
        //        hardwareRobot.elbowtwo, hardwareRobot.claw, hardwareRobot.clawrotation, telemetry);

        this.cameraController = new GeneralCameraController(hardwareRobot.webcamName, telemetry);

        Pose2d initialPos = new Pose2d(0,0,new Rotation2d(0));
        this.robotLocalization = new RobotLocalizationGoBildaPinpoint(initialPos,
                hardwareRobot.Pinpoint,
                opMode,telemetry);
        //this.robotLocalization = new RobotLocalizationFourWheels(hardwareRobot.leftFront,
         //       hardwareRobot.rightFront, hardwareRobot.leftBack, hardwareRobot.rightBack, telemetry);
        /*
        this.inDep = new indepSubsystem(
                opMode,
                drive,
                hardwareRobot
        );*/
        this.telemetry=telemetry;
        this.telemetry.addData("New Robot","Initialized");
        //this.robotDriveController.periodic();
        //this.robotLocalization.periodic();
        this.telemetry.update();
    }

    public void periodic(){
        telemetry.addData("New Robot","Running");
        robotDriveController.periodic();
        robotLocalization.periodic();
        cameraController.periodic();
       // telemetry.update();
    }
}
