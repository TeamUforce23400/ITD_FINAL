package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class SlidesSubsystem extends SubsystemBase {

    //Define motors and servos

//    private PIDController controller;
//
//    private static double p = 0, i = 0, d = 0;
//    private static double f = 0;

    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0;

    public double target;

    private final double ticks_in_degrees = 700/180.0;

    private DcMotorEx verticalSlideMotor1;
    private DcMotorEx verticalSlideMotor2;

    // Define variables
    private int stowedSlidesPosition = 0;
    private int backwardsTransferPosition = 0;
    private int lowChamberPosition = 150;
    private int highChamberPosition = 500;
    private int lowBasketPosition = 800;
    private int highBasketPosition = 2150;

    private int dumpPosition = 800;

    private int dumpHalfPos = 400;

    private int deliverHighChamberPosition = 0;

    //IMPORTANT: this value gives the motor some breathing room on the retraction
    //we can cook motors if this value is too LOW
    //prob only want to make this number larger than 50
    private int STOWED_SLIDE_DIFFERENCE = 50;





    private Telemetry telemetry;

//    public static double kP = 0.01;
//    public static double kI = 0.0;
//    public static double kD = 0.0002;
//    public static double kF = 0.00018;
//
//    private static final PIDFController slidePIDF = new PIDFController(kP, kI, kD, kF);




    public SlidesSubsystem(final HardwareMap hMap,  Telemetry telemetry) {
        verticalSlideMotor1 = hMap.get(DcMotorEx.class, "cr");
        verticalSlideMotor2 = hMap.get(DcMotorEx.class, "cl");
        verticalSlideMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        verticalSlideMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        controller = new PIDController(p, i, d);

//        controller = new PIDController(p, i, d);

        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position Right", verticalSlideMotor1.getCurrentPosition());
        telemetry.addData("Current Position Left", verticalSlideMotor2.getCurrentPosition());
        telemetry.update();


        resetVerticalSlides();

        this.telemetry = telemetry;
    }


    public void resetVerticalSlides(){
        verticalSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void JoySlideUp(){
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSlideMotor1.setPower(1.0);
        verticalSlideMotor2.setPower(1.0);
    }

    public void JoySlideDown(){
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSlideMotor1.setPower(-1.0);
        verticalSlideMotor2.setPower(-1.0);
    }


    public void stowSlides() {
        //verticalSlideMotor.setTargetPosition(stowedSlidesPosition);
        //verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //verticalSlideMotor.setPower(1);
        setSlideTarget(stowedSlidesPosition);
    }

    public void NoPowerSlides(){
        verticalSlideMotor1.setPower(0);
        verticalSlideMotor2.setPower(0);
    }

    public double getCurrentSlidePos(){
        return verticalSlideMotor1.getCurrentPosition();

    }

    public boolean AreSlidesStowed() {
        return verticalSlideMotor1.getCurrentPosition() < (stowedSlidesPosition + STOWED_SLIDE_DIFFERENCE);
    }

    public void highChamberDeliver(){
//        verticalSlideMotor1.setTargetPosition(deliverHighChamberPosition);
//        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor1.setPower(1);
//
//        verticalSlideMotor2.setTargetPosition(deliverHighChamberPosition);
//        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor2.setPower(1);

        setSlideTarget(deliverHighChamberPosition);
    }

    public boolean IsAtHighChamberDeliver(){
        return verticalSlideMotor1.getCurrentPosition() < (deliverHighChamberPosition +50);
    }

    public void backwardsTransfer() {
        verticalSlideMotor1.setTargetPosition(backwardsTransferPosition);
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor1.setPower(1);

        verticalSlideMotor2.setTargetPosition(backwardsTransferPosition);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor2.setPower(1);
    }

    public boolean AreSlidesAllowingBackwardsTransfer() {
        return verticalSlideMotor1.getCurrentPosition() > (backwardsTransferPosition - 50);
    }

    public void lowChamber() {
        verticalSlideMotor1.setTargetPosition(lowChamberPosition);
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor1.setPower(1);

        verticalSlideMotor2.setTargetPosition(lowChamberPosition);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor2.setPower(1);
    }

    public boolean IsAtLowChamber() {
        return verticalSlideMotor1.getCurrentPosition() > (lowChamberPosition - 50);
    }
    public void highChamber() {
//        verticalSlideMotor1.setTargetPosition(highChamberPosition);
//        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor1.setPower(1);
//
//        verticalSlideMotor2.setTargetPosition(highChamberPosition);
//        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor2.setPower(1);

        setSlideTarget(highChamberPosition);
    }

    public boolean IsAtHighChamber() {
        return verticalSlideMotor1.getCurrentPosition() > (highChamberPosition - 50);
    }

    public void lowBasket() {
        /*verticalSlideMotor.setTargetPosition(lowBasketPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);*/
        setSlideTarget(lowBasketPosition);
    }

    public void dumpPosition(){
        setSlideTarget(dumpPosition);
    }

    public boolean IsAtDumpPosition(){
        return verticalSlideMotor1.getCurrentPosition() > (dumpPosition - 50);
    }

    public void halfDump(){
        setSlideTarget(dumpHalfPos);
    }

    public boolean IsAtHalfDumpPosition(){
        return verticalSlideMotor1.getCurrentPosition() > (dumpHalfPos - 50);
    }

    public boolean IsAtLowBasket() {
        return verticalSlideMotor1.getCurrentPosition() > (lowBasketPosition - 50);
    }
    public void highBasket() {
        /*
        verticalSlideMotor.setTargetPosition(highBasketPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
        */
//        setSlideTarget(highBasketPosition);

        controller.setPID(p, i, d);

        int rightPos = verticalSlideMotor1.getCurrentPosition();

        double pid = controller.calculate(rightPos, highBasketPosition);
        double ff = Math.cos(Math.toRadians(highBasketPosition/ticks_in_degrees))*f;

        double power = pid + ff;

        telemetry.addData("Target Position", highBasketPosition);
        telemetry.addData("Current Position right", rightPos);
        telemetry.addData("Current Position left", verticalSlideMotor2.getCurrentPosition());
        telemetry.addData("Motor Power", power);
        telemetry.update();

        verticalSlideMotor1.setPower(power);
        verticalSlideMotor2.setPower(power);
    }

    public boolean IsAtHighBasket() {
        return verticalSlideMotor1.getCurrentPosition() > (highBasketPosition - 50);
    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, highBasketPosition), 0);
        controller.setSetPoint(target);
    }

    public void autoUpdateSlides() {

        controller.setPID(p, i, d);

        int rightPos = verticalSlideMotor1.getCurrentPosition();

        double pid = controller.calculate(rightPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees))*f;

        double power = pid + ff;

        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position right", rightPos);
        telemetry.addData("Current Position left", verticalSlideMotor2.getCurrentPosition());
        telemetry.addData("Motor Power", power);
        telemetry.update();

        verticalSlideMotor1.setPower(power);
        verticalSlideMotor2.setPower(power);

    }

    @Override
    public void periodic() {
        autoUpdateSlides();
    }
}