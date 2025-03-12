package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class OTOSDrive extends MecanumDrive {
    public static class Params {
        public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(0));
        public double linearScalar = 1.0;
        public double angularScalar = 1.0;
    }

    public static OTOSDrive.Params PARAMS = new OTOSDrive.Params();
    public SparkFunOTOS otos;
    private Pose2d lastOtosPose = pose;

    public OTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);

        otos = hardwareMap.get(SparkFunOTOS.class,"sensor_otos");

        //keep this in inches to align with RR
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(PARAMS.offset);
        otos.setPosition(RRPoseToOTOSPose(pose));

        System.out.println(otos.calibrateImu(255, true));
        System.out.println("OTOS calibration complete!");

    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastOtosPose != pose) {
            //ugly
            otos.setPosition(RRPoseToOTOSPose(pose));
        }

        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        pose = OTOSPoseToRRPose(otosPose);
        lastOtosPose = pose;

        // rr standard
        this.poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y),otosVel.h);
    }

    private Pose2d OTOSPoseToRRPose(SparkFunOTOS.Pose2D otosPose){
        return new Pose2d(otosPose.x, otosPose.y, otosPose.h);
    }

    private SparkFunOTOS.Pose2D RRPoseToOTOSPose(Pose2d rrPose){
        return new SparkFunOTOS.Pose2D(rrPose.position.x, rrPose.position.y, rrPose.heading.toDouble());
    }
}