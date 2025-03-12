package org.firstinspires.ftc.teamcode.opmodes.teleop;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Arm", group="Linear OpMode")
@Disabled
public class ArmTest extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor arm = null;
    private DcMotor slide = null;

    private Servo hookLeft;
    private Servo hookRight;

    @Override
    public void runOpMode() {
       // arm  = hardwareMap.get(DcMotor.class, "arm");
        slide = hardwareMap.get(DcMotor.class,"verticalSlidesMotor");

        hookLeft = hardwareMap.get(Servo.class, "ascentLeftHook");
        hookRight = hardwareMap.get(Servo.class, "ascentRightHook");

        DcMotor ascentLeftMotor = hardwareMap.get(DcMotor.class, "ascentLeftMotor");
        DcMotor ascentRightMotor = hardwareMap.get(DcMotor.class, "ascentRightMotor");


        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        //arm.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

           if(gamepad1.a){
             //  arm.setPower(1);
               hookLeft.setPosition(1);
               hookRight.setPosition(1);
           }

            if(gamepad1.b){
                //  arm.setPower(1);
                hookLeft.setPosition(0);
                hookRight.setPosition(0);
            }
           telemetry.addData("Slides", slide.getCurrentPosition());
            telemetry.addData("Arms L", hookLeft.getPosition());
            telemetry.addData("Arms R", hookRight.getPosition());
            telemetry.addData("Hook L", ascentLeftMotor.getCurrentPosition());
            telemetry.addData("Hook R", ascentRightMotor.getCurrentPosition());
           telemetry.update();
        }
    }
}
