package org.firstinspires.ftc.teamcode.DriveController;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;
import org.firstinspires.ftc.teamcode.synchropather.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.translation.TranslationConstants;

/**
 * The class for 4-wheeled Mecanum drive (goBILDA GripForce Strafer Chassis)
 * Similar to DriveSubsystem in TSPMO project
 */
public class RobotDriveController {
    /**
     * Speed Field for power
     */
    public double driveSpeed;
    /**
     * Theta Field for power (direction)
     */
    public double driveTheta;

    /**
     * Velocity Field for power
     */
    public double turnVelocity;

    /**
     * Left front motor/wheel
     */
    private final MotorEx leftFront;

    /**
     * Right front motor/wheel
     */
    private final MotorEx rightFront;

    /**
     * Left back motor/wheel
     */
    private final MotorEx leftBack;

    /**
     * Right back motor/wheel
     */
    private final MotorEx rightBack;
    /**
     * 4-wheeled Mecanum drive
     */
    private final MecanumDrive controller;
    private final Telemetry telemetry;

    private boolean manual;

    public RobotDriveController(MotorEx leftFront,
                                MotorEx rightFront,
                                MotorEx leftBack,
                                MotorEx rightBack, boolean manual,Telemetry telemetry){
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.controller = new MecanumDrive(leftFront,rightFront,leftBack,rightBack);
        this.driveSpeed=0;
        this.driveTheta=0;
        this.turnVelocity=0;
        this.telemetry = telemetry;
        this.manual= manual;
        if (manual == true) {
        setManualWheelMotor(this.rightBack, false);
        setManualWheelMotor(this.rightFront, false);
        setManualWheelMotor(this.leftBack, true);
        setManualWheelMotor(this.leftFront, false);
        }else{
            setAutoWheelMotor(rightBack,false);
            setAutoWheelMotor(rightFront,false);
            setAutoWheelMotor(leftBack,true);
            setAutoWheelMotor(leftFront,false);
        }
        //telemetry.addData("Robot Controller ","Initialized");
        //telemetry.update();
    }

    public void changeInversions() {
        leftFront.setInverted(true);
        rightBack.setInverted(true);
    }

    /**
     * Drives with directions based on robot pov.
     *
     * @param strafe     Strafe power.
     * @param forward     Forward power.
     * @param turn      Turn power.
     */
    public void driveRobotCentricPowers(double strafe, double forward, double turn) {
        double theta;
        double speed = Math.hypot(strafe, forward) * TranslationConstants.MAX_VELOCITY;
        if (speed==0) theta = 0;
        else theta = Math.atan2(forward, strafe);

        driveFieldCentric(theta, speed, -turn * RotationConstants.MAX_ANGULAR_VELOCITY, 0.0);
    }
    /**
     * Drives with directions based on robot pov.
     *
     * @param theta     Direction of drive in radians.
     * @param speed     Desired driving speed in in/s.
     * @param turn      Desired angular velocity in rad/s.
     */
    public void driveRobotCentric(double theta, double speed, double turn) {
        driveFieldCentric(theta, speed, -turn, 0.0);
    }
    /**
     *
     * setting of motor for wheel
     * @param m motor of wheel
     */

    public void setManualWheelMotor(MotorEx m, boolean inv){
        //m.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //m.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setRunMode(Motor.RunMode.RawPower);
        m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m.setInverted(inv);
    }
    public void setAutoWheelMotor(MotorEx m, boolean inv){
        m.setRunMode(Motor.RunMode.VelocityControl);
        m.setInverted(inv);
    }
    /**
     * Corrected driving with bias based on driver pov.
     *
     * @param theta     Direction of drive in radians.
     * @param speed     Desired driving speed in in/s.
     * @param turn      Desired angular velocity in rad/s.
     * @param gyroAngle Robot heading in radians.
     */
    public void driveFieldCentric(double theta,
                                  double speed,
                                  double turn,
                                  double gyroAngle) {
        theta = normalizeAngle(Math.PI/2 + (theta-gyroAngle));
        double maxSpeed = TranslationConstants.MAX_VELOCITY;

        double L = 0;
        double R = 0;
        double theta_w = Math.PI/4; //the angle of the force vector
        if (0<theta && theta<=Math.PI/2) {
            L = 1;
            R = -Math.sin(theta_w-theta) / Math.sin(theta_w+theta);
        }
        else if (Math.PI/2<theta && theta<=Math.PI) {
            L = -Math.sin(theta_w+theta) / Math.sin(theta_w-theta);
            R = 1;
        }
        else if (-Math.PI<=theta && theta<=-Math.PI/2) {
            L = -1;
            R = Math.sin(theta_w-theta) / Math.sin(theta_w+theta);
        }
        else if (-Math.PI/2<theta && theta<=0) {
            L = Math.sin(theta_w+theta) / Math.sin(theta_w-theta);
            R = -1;
        }

        double factor = Math.max(-1, Math.min(1, speed / maxSpeed));
        L *= factor;
        R *= factor;

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = L;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = R;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = R;
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = L;

        turn /= RotationConstants.MAX_ANGULAR_VELOCITY;
        turn = Math.max(-1, Math.min(1, turn));
        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] -= turn;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] += turn;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] -= turn;
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] += turn;

        normalize(wheelSpeeds);


        controller.driveWithMotorPowers(
                wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value],
                wheelSpeeds[RobotDrive.MotorType.kFrontRight.value],
                wheelSpeeds[RobotDrive.MotorType.kBackLeft.value],
                wheelSpeeds[RobotDrive.MotorType.kBackRight.value]
        );
    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        while (radians >= Math.PI) radians -= 2*Math.PI;
        while (radians < -Math.PI) radians += 2*Math.PI;
        return radians;
    }

    /**
     * Normalize the wheel speeds
     */
    private void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }

    /**
     * Drive based on driver pov.
     * @param gyroAngle Robot heading in radians.
     */
    public void driveFieldCentric(double gyroAngle){
        driveFieldCentric(driveTheta,driveSpeed,turnVelocity,gyroAngle);
    }

    /**
     * Stop the motors
     */
    public void stopController(){
        controller.stop();
    }


    public void periodic(){
        telemetry.addData("Drive","  here");
        if (manual == true){
            telemetry.addData("Control","Manual");
        }else{
            telemetry.addData("Control","auto");
        }
    }
}
