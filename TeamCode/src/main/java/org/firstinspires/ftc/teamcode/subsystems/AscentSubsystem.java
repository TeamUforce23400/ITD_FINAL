package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AscentSubsystem extends SubsystemBase {

    private final CRServo leftHook;
    private final CRServo rightHook;
    private final Servo pto;
    private final DcMotorEx ptomotor;

    public AscentSubsystem(HardwareMap hardwareMap) {
        leftHook  = hardwareMap.get(CRServo.class, "hangLeft");
        rightHook = hardwareMap.get(CRServo.class, "hangRight");
        pto = hardwareMap.get(Servo.class, "pto");
        ptomotor = hardwareMap.get(DcMotorEx.class, "ptomotor");


        // If one spins backwards, flip its direction here:
         rightHook.setDirection(CRServo.Direction.REVERSE);

        // start stopped
        stop();
    }

    /**
     * Spins both servos forward until you call stop()
     */
    public void moveForward() {
        leftHook.setPower(1.0);
        rightHook.setPower(1.0);
    }

    /**
     * Spins both servos in reverse until you call stop()
     */
    public void moveBackward() {
        leftHook.setPower(-1.0);
        rightHook.setPower(-1.0);
    }

    /**
     * Immediately stops both servos
     */
    public void stop() {
        leftHook.setPower(0.0);
        rightHook.setPower(0.0);
    }

    public void ptoBase() {
        pto.setPosition(0.25);

    }

    public void ptoSwitch() {
        pto.setPosition(0.35);

    }

    public void ptoMotorStop() {
        ptomotor.setPower(0);

    }

    public void ptoMotorDown() {
        ptomotor.setPower(-1);

    }
}
