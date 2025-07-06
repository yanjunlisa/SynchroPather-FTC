package org.firstinspires.ftc.teamcode.synchropather.translation;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveController.RobotDriveController;
import org.firstinspires.ftc.teamcode.DriveController.RobotLocalization;
import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;
import org.firstinspires.ftc.teamcode.synchropather.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.__util__.superclasses.Plan;

import java.util.ArrayList;

/**
 * Object containing a sequence of Movements for translational drive.
 */
@Config
public class TranslationPlan extends Plan<TranslationState> {

	//4-wheel Mecanum drivetrain, 2D PIDF controller(SQUIDF)
	//Controller Constants

	//FeedForward constants
	public static double kS=0;
	public static double kV=1;
	public static double kA=0;

	//Positional SQUID constants
	public static double kSQU=0;
	public static double kI=0;
	public static double kD=0;

	//controller variables

	private double intexdt=0;
	private double inteydt=0;

	//error history array
	private final ArrayList<Double> exHistory;
	private final ArrayList<Double> eyHistory;
	private final ArrayList<Double> dtHistory;

	private static int sizeError = 5;

	//For calculating derivative
	private ElapsedTime runtime;


	private final RobotDriveController robotDriveController;

	private final RobotLocalization robotLocalization;
	/**
	 * Creates a new TranslationPlan object with the given Movements.
	 * @param movements
	 */
	public TranslationPlan(RobotDriveController robotDriveController,
						   RobotLocalization robotLocalization,
						   Movement... movements)
	{
		super(MovementType.TRANSLATION, movements);
		this.robotDriveController=robotDriveController;
		this.robotLocalization=robotLocalization;
		this.exHistory= new ArrayList<Double>();
		this.eyHistory = new ArrayList<Double>();
		this.dtHistory = new ArrayList<Double>();
	}

	/**
	 * Controls the translation output of the robot to the TranslationState at the elapsedTime.
	 */

	@Override
	public void loop() {

		//TODO: Write Controller
		TranslationState desiredState = getCurrentState();
		TranslationState desiredVelocity = getCurrentVelocity();
		TranslationState desiredAcceleration = getCurrentAcceleration();
		double dxv = desiredVelocity.getX();
		double dyv = desiredVelocity.getY();
		double dxa = desiredAcceleration.getX();
		double dya = desiredAcceleration.getY();

		//current state
		Pose2d currentPos = robotLocalization.getPose();
		double currentX = currentPos.getX();
		double currentY = currentPos.getY();
		TranslationState currentState=new TranslationState(currentX, currentY);

		//state error
		TranslationState error = desiredState.minus(currentState);
	    TranslationState u = updateHistory(error,dxv, dxa, dyv, dya);

		//set drive parameters
		robotDriveController.driveTheta=u.theta();
		robotDriveController.driveSpeed=u.hypot();
		robotDriveController.periodic();
		robotDriveController.driveFieldCentric(currentPos.getHeading());

	}
	/*
	public void loop(){
		robotDriveController.driveTheta= TranslationConstants.MAX_ACCELERATION;
		robotDriveController.driveSpeed=TranslationConstants.MAX_VELOCITY;
		robotDriveController.driveFieldCentric(0.0);
	}*/


	public double stencil(ArrayList<Double> a){
		double averageDeltaTime = dtHistory.stream().mapToDouble(aa->aa).average().orElse(0);
		return (-a.get(4)+8*a.get(3)-8*a.get(1)+a.get(0))/(12*averageDeltaTime);
	}
	public TranslationState updateHistory(TranslationState error, double dxv, double dxa, double dyv, double dya){
		double ex=error.getX();
		double ey=error.getY();
		exHistory.add(ex);
		eyHistory.add(ey);
		if (exHistory.size()>sizeError) exHistory.remove(0);
		if (eyHistory.size()>sizeError) eyHistory.remove(0);

		double deltaTime;
		boolean runtimeWasNull= false;
		if (runtime == null){
			runtime= new ElapsedTime(0);
			deltaTime=0;
			runtimeWasNull= true;
		}else{
			deltaTime = runtime.seconds();
			runtime.reset();
			dtHistory.add(deltaTime);
			if (dtHistory.size()>sizeError)
				dtHistory.remove(0);

		}
		if (!runtimeWasNull){
			intexdt+=deltaTime*ex;
			inteydt+=deltaTime*ey;
		}

		//flush the stack
		if(exHistory.size()>1){
			if (exHistory.get(exHistory.size()-2)*ex<=0){
				intexdt=0;
			}
		}
		if(eyHistory.size()>1){
			if (eyHistory.get(eyHistory.size()-2)*ey<=0){
				inteydt=0;
			}
		}

		if(kI !=0){
			double integralPowerThreshold=0.25;
			double integralThresholdBound =
					Math.abs(integralPowerThreshold*TranslationConstants.MAX_PATHING_VELOCITY/kI);
			intexdt = Math.max(-integralThresholdBound,Math.min(integralThresholdBound,intexdt));
			inteydt = Math.max(-integralThresholdBound,Math.min(integralThresholdBound,inteydt));
		}
		//error derivatives
		double dexdt=0;
		if (dtHistory.size()==sizeError){
			if(exHistory.size()==sizeError){
				dexdt=stencil(exHistory);
			}
		}
		double deydt=0;
		if (dtHistory.size()==sizeError){
			if(eyHistory.size()==sizeError){
				deydt=stencil(eyHistory);
			}
		}

		//Positional SQUID
		double squx = Math.signum(ex)*Math.sqrt(Math.abs(ex));
		double squy = Math.signum(ey)*Math.sqrt(Math.abs(ey));
		double utx = kSQU*squx+kI*intexdt + kD*dexdt;
		double uty = kSQU*squy+kI*inteydt + kD*deydt;
		TranslationState ut = new TranslationState(utx,uty);

		//FeedForward
		double fux = kV*dxv+kA *dxa;
		double fuy = kV*dyv +kA * dya;
		TranslationState fu = new TranslationState(fux,fuy);
		TranslationState fu_static = new TranslationState(kS, fu.theta(),true).times(Math.signum(fu.hypot()));
		//if fu is zero, then fu_static should be 0
		fu=fu.plus(fu_static);
		TranslationState u = ut.plus(fu);
		return u;
	}

	@Override
	public void stop() {

		//TODO: Halt Subsystem
		robotDriveController.driveTheta=0;
		robotDriveController.driveSpeed=0;
		robotDriveController.stopController();

	}

	@Override
	public String toString(){
		return "Translation Plan";
	}

}