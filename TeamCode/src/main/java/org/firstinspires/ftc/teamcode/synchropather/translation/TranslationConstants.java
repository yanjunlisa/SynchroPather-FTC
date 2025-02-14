package org.firstinspires.ftc.teamcode.synchropather.translation;

/**
 * A static class used by Translation Movements to reference important kinematic and algorithmic tuning values.
 */
public class TranslationConstants {

    /**
     *  Max velocity of the robot used for SynchroPather in in/s.
     */
    public static double MAX_VELOCITY = 50;
    /**
     *  Max acceleration of the robot in in/s^2.
     */
    public static double MAX_ACCELERATION = 50;

    /**
     *  Used for differentiating and integrating spline paths, between 0 and 1 (lower = more calculations, more detail).
     */
    public static final double delta_t = 0.005;

}
