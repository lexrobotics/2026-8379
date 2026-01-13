package com.example.MeepMeepTesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // variable initialization
    static double shootingTime = 3;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // TO-DO: replace with real constraint values!! they need to be consistent with MecanumDrive.java
        // note: trackWidth is not robot width, it is about the distance btwn centers of left and right wheels, usually found exactly by tuning (ahem)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // TO-DO: actually test!! my main concern rn is that it doesnt get enough time to pick up balls
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49, 49, Math.toRadians(125)))
                // getting off the starting line + shooting preloaded (0th) triple
                .strafeTo(new Vector2d(-40, 40))
                .waitSeconds(shootingTime)
                // sequence to pick up another (1st) triple
                .splineToLinearHeading(new Pose2d(-11.6, 20, Math.toRadians(90)), Math.toRadians(45))
                .strafeTo(new Vector2d(-11.6, 55))
                // sequence to shoot 1st triple
                .splineToLinearHeading(new Pose2d(-40, 40, Math.toRadians(125)), Math.toRadians(125))
                .waitSeconds(shootingTime)
                // sequence to pick up 2nd triple
                .splineToLinearHeading(new Pose2d(11.6, 20, Math.toRadians(90)), Math.toRadians(45))
                .strafeTo(new Vector2d(11.6, 52))
                // sequence to shoot 2nd triple
                .splineToLinearHeading(new Pose2d(-40, 40, Math.toRadians(125)), Math.toRadians(125))
                .waitSeconds(shootingTime)
                // sequence to pick up 3rd triple
                .splineToLinearHeading(new Pose2d(34.6, 20, Math.toRadians(90)), Math.toRadians(45))
                .strafeTo(new Vector2d(34.6, 49))
                // sequence to shoot 3rd triple
                .splineToLinearHeading(new Pose2d(-40, 40, Math.toRadians(125)), Math.toRadians(125))
                .waitSeconds(shootingTime)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}