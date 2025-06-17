package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Cascade Check")
public class CascadeCheck extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void init(){
        leftMotor = hardwareMap.get(DcMotor.class, "cl");
        rightMotor = hardwareMap.get(DcMotor.class, "cr");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        leftMotor.setPower(drive);
        rightMotor.setPower(drive);
    }
}
