package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBasket {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(500,120);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17,17.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-14.8, -64, Math.toRadians(-90)))

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-8.5, -35, Math.toRadians(-90)), Math.toRadians(90))

                //drop off specimen
                        .setTangent(Math.toRadians(-131))
                        .splineToLinearHeading(new Pose2d(-35,-50,Math.toRadians(-235)),Math.toRadians(-131))
                //pick up the first sample
                 //       .setTangent(Math.toRadians(180))
                //    .splineToLinearHeading(new Pose2d(-55,-50,Math.toRadians(-315)),Math.toRadians(180))
                //drop off in the basket
                  /*      .setTangent(Math.toRadians(90))
                     .splineToLinearHeading(new Pose2d(-58,-40,Math.toRadians(-270)),Math.toRadians(90))

                        .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55,-50,Math.toRadians(-315)),Math.toRadians(-90))
                //drop off in the basket

                        .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58,-40,Math.toRadians(-235)),Math.toRadians(90))

                        .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-53,-50,Math.toRadians(-315)),Math.toRadians(-90))

                        .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-30,-10,Math.toRadians(180)),Math.toRadians(0))
*/

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}