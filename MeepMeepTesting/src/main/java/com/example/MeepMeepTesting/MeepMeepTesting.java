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

        double shootX = -38.0;
        double shootY = 40.0;
        double shootAngle = Math.toRadians(135);
        double lineupY = 20.0;
        double inOneY = 50.0;
        double inOneX = -12.0;
        double inTwoX = 12.0;
        double inTwoY = 60.0;
        double straightAngle = Math.toRadians(90);

        Pose2d beginPose = new Pose2d(-49, 49, Math. toRadians(125));

        //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Pose2d pow = new Pose2d(shootX, shootY, shootAngle);
        MeepMeep meepMeep = new MeepMeep(500);

        // TO-DO: replace with real constraint values!! they need to be consistent with MecanumDrive.java
        // note: trackWidth is not robot width, it is about the distance btwn centers of left and right wheels, usually found exactly by tuning (ahem)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

       // Pose2d beginPose = new Pose2d(new Vector2d(0, 0), Math. toRadians(120));

        //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Pose2d lastPose = beginPose;

        // TO-DO: actually test!! my main concern rn is that it doesnt get enough time to pick up balls
        myBot.runAction(myBot.getDrive().actionBuilder(lastPose)
                //.stopAndAdd(new HungryHippo()) //intake and trans always on
                .strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle)//getting into pos
                .waitSeconds(5)//.stopAndAdd(new MichaelJordan())
                .strafeToSplineHeading(new Vector2d(inOneX, lineupY), Math.toRadians(90))   //lineup to eat
                .waitSeconds(0.2)
                .strafeToSplineHeading(new Vector2d(inOneX, inOneY), Math.toRadians(90.00)) //eat
                .strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle) //shooting second
                .waitSeconds(5)//.stopAndAdd(new MichaelJordan())
                .strafeToSplineHeading(new Vector2d(inTwoX, lineupY),  Math.toRadians(90.00)) //lineup to eat
                .strafeToSplineHeading(new Vector2d(inTwoX, inTwoY), Math.toRadians(90)) //eat
                .strafeToSplineHeading(new Vector2d(inTwoX, inOneY),  Math.toRadians(90.00)) //back up a tad
                .strafeToSplineHeading(new Vector2d(shootX, shootY), shootAngle)//shoot third
                .waitSeconds(5)//.stopAndAdd(new MichaelJordan())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}