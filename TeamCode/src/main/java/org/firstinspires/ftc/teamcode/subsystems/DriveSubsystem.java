package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.OTOSDrive;

public class DriveSubsystem extends SubsystemBase {

    //private MecanumDrive mecanum;

    private MecanumDrive drive;

    private Telemetry telemetry;

    public DriveSubsystem(final HardwareMap hMap, Telemetry telemetry){

        this.telemetry = telemetry;

        drive = new MecanumDrive(hMap,new Pose2d(0,0,0));


    }

    public void resetHeading(){
        drive.pose = new Pose2d(0,0,0);
    }

    public void drive(double leftX, double leftY, double rightX, double scale){
        Pose2d poseEstimate = drive.pose;
        double heading = drive.pose.heading.toDouble();

        Vector2d sticks = new Vector2d(
                leftY * scale,
                -leftX * scale
        );

        double rotX = sticks.x * Math.cos(-heading) - sticks.y * Math.sin(-heading);
        double rotY = sticks.x * Math.sin(-heading) + sticks.y * Math.cos(-heading);

        Vector2d updatedVector = new Vector2d(rotX, rotY);

        drive.setDrivePowers(new PoseVelocity2d(
                updatedVector,
                -rightX
        ));


        drive.updatePoseEstimate();


    }

    public void setPose(Pose2d currentPose){
        drive.pose = currentPose;
    }
}
