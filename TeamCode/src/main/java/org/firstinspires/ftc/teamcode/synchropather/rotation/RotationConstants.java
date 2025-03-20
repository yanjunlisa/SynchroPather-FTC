package org.firstinspires.ftc.teamcode.synchropather.rotation;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Rotation Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class RotationConstants {

    /**
     *  Max angular velocity of the robot in rad/s.
     */
    public static double MAX_ANGULAR_VELOCITY = 4; //TODO: TUNE

    /**
     *  Max angular velocity of the robot used for SynchroPather in rad/s.
     */
    public static double MAX_PATHING_ANGULAR_VELOCITY = 0.9*MAX_ANGULAR_VELOCITY; //TODO: TUNE

    /**
     *  Max angular acceleration of the robot in rad/s^2.
     */
    public static double MAX_ANGULAR_ACCELERATION = 4; //TODO: TUNE

}