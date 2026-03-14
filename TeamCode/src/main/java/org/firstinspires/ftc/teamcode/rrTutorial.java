package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="rrTest1")
public class rrTutorial extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        com.acmerobotics.roadrunner.ftc.Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, -60, Math.toRadians(0)))
                .splineTo(new Vector2d(35,-30),Math.toRadians(90))
                .strafeTo(new Vector2d(35,-10))
                .strafeTo(new Vector2d(45,-15))
                .strafeTo(new Vector2d(45,-65))
                .waitSeconds(0)
                .splineTo(new Vector2d(50,-5),Math.toRadians(90))
                .strafeTo(new Vector2d(60,-5))
                .setReversed(true)
                .splineTo(new Vector2d(60,-65),Math.toRadians(270))
                .setReversed(false)
                .strafeTo(new Vector2d(60,-20))
                .splineTo(new Vector2d(60,-5),Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(60,-65),Math.toRadians(270)).build());

    }
}
