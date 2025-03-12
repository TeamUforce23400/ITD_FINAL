package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferSubsystem extends SubsystemBase {

    //Define motors and servos
    private Servo armLeftServo;
    private Servo armRightServo;
    private Servo gripplerServo;
    private Servo griggleWristServo;

    private DigitalChannel magnetSensor;  // Digital channel Object

    // Define variables
    private double backwardsTransferPosition = 0;
    // private double stowedTransferPosition = 0;//higher number is out further from slides
    private double stowedPosition = 0.45;
    private double flippedPosition = 0.4; //higher number is harder on the bar and padding
    private double middleGripplerRotation = 0.5;
    private double leftGripplerRotation = 0;
    private double rightGripplerRotation = 1;
    private double closedGripplerPosition = 0;
    private double openGripplerPosition = 0.5;

    public TransferSubsystem(final HardwareMap hMap) {
        armLeftServo = hMap.get(Servo.class, "opl");
        armRightServo = hMap.get(Servo.class, "opr");
        gripplerServo = hMap.get(Servo.class, "claw");
        griggleWristServo = hMap.get(Servo.class, "griggleWrist");
        magnetSensor = hMap.get(DigitalChannel.class, "magnet");

        magnetSensor.setMode(DigitalChannel.Mode.INPUT);

        armLeftServo.setDirection(Servo.Direction.FORWARD);
        armRightServo.setDirection(Servo.Direction.REVERSE);
        gripplerServo.setDirection(Servo.Direction.REVERSE);
        stowTransfer();
        griggleWristServo.setPosition(0.5);
        openGrippler();
    }

    public void backwardsTransfer() {
        armLeftServo.setPosition(backwardsTransferPosition);
        armRightServo.setPosition(backwardsTransferPosition);
    }

    public boolean IsTransferBackwards() {return true;}

    public void stowTransfer() {
        armLeftServo.setPosition(stowedPosition);
        armRightServo.setPosition(stowedPosition);
    }

    public boolean IsTransferStowed() {
        return true;
    }

    public void flipTransfer() {
        armLeftServo.setPosition(flippedPosition+0.2);
        armRightServo.setPosition(flippedPosition);
    }

    public boolean IsTransferClosed(){
        return !magnetSensor.getState();
    }

    public boolean IsTransferFlipped() {
        return true;
    }

    public void middleGripplerRotation() {
        griggleWristServo.setPosition(middleGripplerRotation);
    }

    public boolean IsGripplerAtMiddleRotation() {
        return true;
    }

    public void leftGripplerRotation() {
        griggleWristServo.setPosition(leftGripplerRotation);
    }

    public boolean IsGripplerAtLeftRotation() {
        return true;
    }

    public void rightGripplerRotation() {
        griggleWristServo.setPosition(rightGripplerRotation);
    }

    public boolean IsGripplerAtRightRotation() {
        return true;
    }

    public void openGrippler() {
        gripplerServo.setPosition(openGripplerPosition);
    }

    public boolean IsGripplerOpen() {
        return true;
    }

    public void closeGrippler() {
        gripplerServo.setPosition(closedGripplerPosition);
    }

    public boolean IsGripplerClosed() {
        return true;
    }
}