package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;

public class ExampleSynchroPatherAuto extends LinearOpMode {

    Synchronizer synchronizer;

    @Override
    public void runOpMode() throws InterruptedException {
        initSynchronizer();

        waitForStart();

        synchronizer.start();
        while (opModeIsActive() && !synchronizer.update());
        synchronizer.stop();
    }


    private void initSynchronizer() {
        // Translation plan
        CRSplineTranslation spline = new CRSplineTranslation(0,
                new TranslationState(0, 24),
                new TranslationState(24, 0),
                new TranslationState(0, -24),
                new TranslationState(-24, 0),
                new TranslationState(0, 24)
        );
        TranslationPlan translationPlan = new TranslationPlan(
                spline
        );

        // Rotation plan
        LinearRotation rotation = new LinearRotation(new TimeSpan(0, spline.getEndTime()),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(360))
        );
        RotationPlan rotationPlan = new RotationPlan(
                rotation
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                translationPlan,
                rotationPlan
        );
    }

}
