package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepCoach {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(
                        //new Pose2d(-48, -48, Math.toRadians(45))).lineToX(0).turn(Math.toRadians(90)).lineToY(30).turn(Math.toRadians(90)).lineToX(0).turn(Math.toRadians(90)).lineToY(0).turn(Math.toRadians(90))
                        new Pose2d(-51.5, -52.5, (Math.toRadians(45))))
                .strafeTo(new Vector2d(-47.26,-48.26))
                .waitSeconds(4.5)
                .strafeTo(new Vector2d(-20,-20))
                .strafeToConstantHeading(new Vector2d(67,67)).build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}