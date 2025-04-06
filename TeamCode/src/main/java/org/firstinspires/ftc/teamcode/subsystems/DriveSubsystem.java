package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Telemetry telemetry;
    private static final double DEADZONE = 0.05; // Adjust as needed

    public DriveSubsystem(final HardwareMap hMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        frontLeft = hMap.get(DcMotor.class, "fl");
        frontRight = hMap.get(DcMotor.class, "fr");
        backLeft = hMap.get(DcMotor.class, "bl");
        backRight = hMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double leftX, double leftY, double rightX, double scale) {
        double forward = Math.abs(leftY) > DEADZONE ? leftY * scale : 0;
        double strafe = Math.abs(leftX) > DEADZONE ? leftX * scale : 0;
        double rotate = Math.abs(rightX) > DEADZONE ? rightX * scale : 0;

        double flPower = forward + strafe + rotate;
        double frPower = forward - strafe - rotate;
        double blPower = forward - strafe + rotate;
        double brPower = forward + strafe - rotate;

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }
}
