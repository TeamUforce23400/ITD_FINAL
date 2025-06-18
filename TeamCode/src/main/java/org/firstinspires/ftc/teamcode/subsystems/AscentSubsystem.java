package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AscentSubsystem extends SubsystemBase {

    private final CRServo leftHook;
    private final CRServo rightHook;

    public AscentSubsystem(HardwareMap hardwareMap) {
        leftHook  = hardwareMap.get(CRServo.class, "hangLeft");
        rightHook = hardwareMap.get(CRServo.class, "hangRight");

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
}
