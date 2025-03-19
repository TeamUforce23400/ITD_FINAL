package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp (name = "Slides Test")
public class SlidesTest extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degrees = 700/180.0;

    private DcMotorEx cas1;
    private DcMotorEx cas2;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        cas1 = hardwareMap.get(DcMotorEx.class, "cr");
        cas2 = hardwareMap.get(DcMotorEx.class, "cl");
        cas1.setDirection(DcMotorEx.Direction.FORWARD);
        cas2.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int caspos = cas1.getCurrentPosition();
        double pid = controller.calculate(caspos, target);


        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees))*f;

        double power = pid + ff;

        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position right", caspos);
        telemetry.addData("Current Position left", cas2.getCurrentPosition());
        telemetry.addData("Motor Power", power);
        telemetry.update();

        cas1.setPower(power);
        cas2.setPower(power);
    }


}
