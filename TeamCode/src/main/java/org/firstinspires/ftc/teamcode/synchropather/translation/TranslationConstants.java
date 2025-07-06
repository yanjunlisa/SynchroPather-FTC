package org.firstinspires.ftc.teamcode.synchropather.translation;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Translation Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class TranslationConstants {

    /**
     *  Max velocity of the robot in in/s.
     */
    public static double MAX_VELOCITY = 10;
    /**
     *  Max velocity of the robot used for SynchroPather in in/s.
     */
    public static double MAX_PATHING_VELOCITY = 0.9*MAX_VELOCITY;
    /**
     *  Max acceleration of the robot in in/s^2.
     */
    public static double MAX_ACCELERATION = 10;

    /**
     *  Used for differentiating and integrating spline paths, between 0 and 1 (lower = more calculations, more detail).
     */
    public static final double delta_t = 0.005;

}
