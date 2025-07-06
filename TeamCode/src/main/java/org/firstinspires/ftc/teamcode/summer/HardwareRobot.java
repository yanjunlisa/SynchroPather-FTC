package org.firstinspires.ftc.teamcode.summer;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspire.ftc.teamcode.subsystems.GoBildaPinpointDriver;

/**
 * The class for all hardware components,
 * each component is recognized in this class.
 */
public class HardwareRobot {
    /**
     * a left front motor
     */
    public final MotorEx leftFront;
    /**
     * a right front motor
     */
    public final MotorEx rightFront;
    /**
     * a left back motor
     */
    public final MotorEx leftBack;
    /**
     * a right back motor
     */
    public final MotorEx rightBack;

    /**
     * a claw servo
     */
    public final ServoImplEx claw;
    /**
     * a left arm motor
     */
    public final MotorEx elbow;
    /**
     * a right arm motor
     */
    public final MotorEx elbowtwo;

    /**
     * a wrist servo
     */
    public final ServoImplEx clawrotation;
    /**
     * a webcam
     */
    public final WebcamName webcamName;

    public HardwareRobot(HardwareMap hardwareMap) {

        ////////////
        // WHEELS //
        ////////////
        leftFront = new MotorEx(hardwareMap,"leftFront",Motor.GoBILDA.RPM_312);
        rightFront = new MotorEx(hardwareMap,"rightFront",Motor.GoBILDA.RPM_312);
        leftBack = new MotorEx(hardwareMap,"leftBack",Motor.GoBILDA.RPM_312);
        rightBack = new MotorEx(hardwareMap,"rightBack",Motor.GoBILDA.RPM_312);
        //check rpm for elbow
        elbow = new MotorEx(hardwareMap, "Elbow", Motor.GoBILDA.RPM_30);
        elbowtwo = new MotorEx(hardwareMap, "elbowtwo", Motor.GoBILDA.RPM_30);
        /////////////
        // SERVOS  //
        /////////////
        claw = hardwareMap.get(ServoImplEx.class, "Claw");
        clawrotation = hardwareMap.get(ServoImplEx.class, "Rotation");

        //for visionSubSystem
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    }
}