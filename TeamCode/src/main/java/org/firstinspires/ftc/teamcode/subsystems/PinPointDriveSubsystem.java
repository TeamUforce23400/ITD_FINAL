package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.OTOSDrive;
import org.firstinspires.ftc.teamcode.utils.PinpointDrive;

public class PinPointDriveSubsystem extends SubsystemBase {

    private PinpointDrive mecanum;

    public PinPointDriveSubsystem(final HardwareMap hMap){


        mecanum = new PinpointDrive(hMap, new Pose2d(0,0,0));


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
