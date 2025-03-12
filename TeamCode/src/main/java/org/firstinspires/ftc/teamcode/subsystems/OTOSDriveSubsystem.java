package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.OTOSDrive;

public class OTOSDriveSubsystem extends SubsystemBase {

    private OTOSDrive mecanum;
    RevIMU imu;


    public OTOSDriveSubsystem(final HardwareMap hMap){

       // imu = new RevIMU(hMap);

        mecanum = new OTOSDrive(hMap, new Pose2d(0,0,0));

       // imu.init();
    }

    public void drive(double leftX, double leftY, double rightX){
        mecanum.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -leftY,
                        -leftX
                ),
                -rightX
        ));

        mecanum.updatePoseEstimate();
    }
}
