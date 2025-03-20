package org.firstinspires.ftc.teamcode.synchropather.translation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.synchropather.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Plan;

/**
 * Object containing a sequence of Movements for translational drive.
 */
@Config
public class TranslationPlan extends Plan<TranslationState> {

	/**
	 * Creates a new TranslationPlan object with the given Movements.
	 * @param movements
	 */
	public TranslationPlan(Movement... movements) {
		super(MovementType.TRANSLATION, movements);
	}

	/**
	 * Controls the translation output of the robot to the TranslationState at the elapsedTime.
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