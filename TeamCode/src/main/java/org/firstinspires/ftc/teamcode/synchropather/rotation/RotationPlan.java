package org.firstinspires.ftc.teamcode.synchropather.rotation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.synchropather.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Plan;

/**
 * Object containing a sequence of Movements for rotational drive.
 */
@Config
public class RotationPlan extends Plan<RotationState> {

	/**
	 * Creates a new RotationPlan object with the given Movements.
	 * @param movements
	 */
	public RotationPlan(Movement... movements) {
		super(MovementType.ROTATION, movements);
	}

	/**
	 * Controls the rotation output of the robot to the RotationState at the elapsedTime.
	 */
	@Override
	public void loop() {

		//TODO: Write Controller

	}

	@Override
	public void stop() {

		//TODO: Halt Subsystem

	}

}