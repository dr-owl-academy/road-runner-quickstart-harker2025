package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

      

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, 20, 0))
                .lineToX(-56)
                .turn(Math.toRadians(-45))
                .lineToX(-32)
                .strafeTo(new Vector2d(-32, 35))

                // Wait and then turn to face right (0 degrees)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(45)) // Robot is now at 90-degree heading

                // FIX: Turn to face the correct direction (0 degrees) before moving along X
                .turn(Math.toRadians(-90))
                .lineToX(56)

                // FIX: Turn to face down (-90 degrees) before moving along Y
                .turn(Math.toRadians(-90))
                .lineToY(20)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
