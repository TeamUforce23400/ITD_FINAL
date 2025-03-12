package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
@TeleOp (name = "ServoTestwithPWM")
public class ServoPWMTuning extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;
    private FtcDashboard dashboard;

    public static double servoPos = 0.5;
    public static double minPWM = 500;
    public static double maxPWM = 2500;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "opr");
        servo2 = hardwareMap.get(Servo.class, "opl");

        dashboard = FtcDashboard.getInstance();
        setServoPWMRange(servo1, minPWM, maxPWM);
        setServoPWMRangee(servo2, minPWM, maxPWM);

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()){
            setServoPWMRange(servo1, minPWM, maxPWM);
            setServoPWMRangee(servo2, minPWM, maxPWM);


            servo1.setPosition(servoPos);
            servo2.setPosition(servoPos-0.2);

            telemetry.addData("servo pos", servoPos);
            telemetry.addData("MIN PWM", minPWM);
            telemetry.addData("MAX PWM", maxPWM);
            telemetry.update();
        }

    }

    private void setServoPWMRange(Servo servo, double minPWM, double maxPWM) {
        if(servo.getController() instanceof PwmControl){
            ((PwmControl) servo.getController()).setPwmRange((new PwmControl.PwmRange(minPWM, maxPWM)));
        }
    }

    private void setServoPWMRangee(Servo servoo, double minPWM, double maxPWM) {
        if(servoo.getController() instanceof PwmControl){
            ((PwmControl) servoo.getController()).setPwmRange((new PwmControl.PwmRange(minPWM, maxPWM)));
        }
    }
}
