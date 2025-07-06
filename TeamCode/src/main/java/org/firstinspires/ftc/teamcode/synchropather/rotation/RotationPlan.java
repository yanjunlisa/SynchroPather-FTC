package org.firstinspires.ftc.teamcode.synchropather.rotation;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.synchropather.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Plan;

import java.util.ArrayList;

/**
 * Object containing a sequence of Movements for rotational drive.
 */
@Config
public class RotationPlan extends Plan<RotationState> {

	// contriller constants
	public static double kS = 0;
	public static double kV = 1;
	public static double kA =0;

	public static double kSQU=0;
	public static double kI = 0;
	public static double kD =0;

	//Controller variable
	private double intedt = 0;

	//error history array
	private final ArrayList<Double> eHistory;
	private final ArrayList<Double> dtHistory;

	private static int sizeError = 5;
	private ElapsedTime runtime;

	//hardware
	private final RobotDriveController robotDriveController;
	private final RobotLocalization robotLocalization;
	/**
	 * Creates a new RotationPlan object with the given Movements.
	 * @param movements
	 */
	public RotationPlan(RobotDriveController robotDriveController,
						RobotLocalization robotLocalization,
						Movement... movements) {

		super(MovementType.ROTATION, movements);
		this.robotDriveController = robotDriveController;
		this.robotLocalization = robotLocalization;
		this.eHistory= new ArrayList<Double>();
		this.dtHistory = new ArrayList<Double>();
	}

	/**
	 * Controls the rotation output of the robot to the RotationState at the elapsedTime.
	 */
	@Override
	public void loop() {
		RotationState desiredState = getCurrentState();
		RotationState desiredVelocity = getCurrentVelocity();
		RotationState desiredAcceleration = getCurrentAcceleration();
		double dv = desiredVelocity.getHeading();
		double da = desiredAcceleration.getHeading();

		//current state
		Pose2d currentPose = robotLocalization.getPose();
		RotationState currentState = new RotationState(currentPose.getHeading());

		RotationState error = desiredState.minus(currentState);
		double e = normalizeAngle(error.getHeading());
		eHistory.add(e);
		if(eHistory.size()>sizeError) eHistory.remove(0);

		double deltaTime;
		boolean runtimeWasNull = false;
		if(runtime == null){
			runtime = new ElapsedTime(0);
			deltaTime=0;
			runtimeWasNull=true;
		}else{
			deltaTime = runtime.seconds();
			runtime.reset();
			dtHistory.add(deltaTime);
			if (dtHistory.size()>sizeError) dtHistory.remove(0);
		}

		if(!runtimeWasNull){
			intedt +=deltaTime*e;
		}
		if (eHistory.size()>1){
			if(eHistory.get(eHistory.size()-2)*e<=0){
				intedt=0;
			}
		}if(kI!=0){
			double integralPowerThreshold = 0.25;
			double integralThresholdBound = Math.abs(integralPowerThreshold*RotationConstants.MAX_ANGULAR_VELOCITY/kI);
			intedt = bound(intedt,-integralThresholdBound,integralThresholdBound);
		}

		//error derivative
		double dedt=0;
		if(dtHistory.size()==sizeError){
			if(eHistory.size()==sizeError){
				dedt=stencil(eHistory);
			}
		}
		//rotational squid
		double u=0;
		double squ=Math.signum(e)*Math.sqrt(Math.abs(e));
		u+=kSQU*squ + kI*intedt+kD*dedt;

		//FeedForward
		u+=kS*Math.signum(dv)+kV*dv+kA*da;

		//set drive parameters
		robotDriveController.turnVelocity=u;
		robotDriveController.driveFieldCentric(currentPose.getHeading());
	}

	/**
	 * Normalize a given angle to (-pi, pi] radians
	 *
	 * @param radians the given angle in radians
	 * @return the normalized angle in radians
	 */
	private static double normalizeAngle(double radians){
		while (radians>=Math.PI) radians -=2*Math.PI;
		while (radians<-Math.PI) radians+=2*Math.PI;
		return radians;
	}

	/**
	 * Compute the derivative of a time-series signal using the
	 * five-point stencil finite difference method.
	 * this is for size 5 history data.
	 * The middle point is skipped.
	 * @param a The process value array
	 * @return Approximated derivative according to the
	 * Five-point stencil.
	 */
	public double stencil(ArrayList<Double> a){
		if (a.size()<sizeError || dtHistory.isEmpty()) return 0;
		double averageDeltaTime = dtHistory.stream().mapToDouble(aa->aa).average().orElse(0);
		return (-a.get(4)+8*a.get(3)-8*a.get(1)+a.get(0))/(12*averageDeltaTime);
	}

	private static double bound(double x, double lower, double upper){
		return Math.max(lower, Math.min(upper,x));
	}
	@Override
	public void stop() {

		//TODO: Halt Subsystem
		robotDriveController.turnVelocity=0;
		robotDriveController.stopController();

	}

	@Override
	public String toString(){
		return "Rotation Plan";
	}

}