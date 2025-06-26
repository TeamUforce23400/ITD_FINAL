package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawCloseCommand;
import org.firstinspires.ftc.teamcode.commands.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
//import org.firstinspires.ftc.teamcode.commands.FireOneShotCommand;
import org.firstinspires.ftc.teamcode.commands.FireOneShotCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawYawBaseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawYawSecondCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorBlueCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorBlueOrNeutralCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorNeutralCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorRedCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorRedOrNeutralCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotIntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotMid;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesFrontTransferCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInTransferCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutHalfCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeResetCommand;
import org.firstinspires.ftc.teamcode.commands.ResetArmForTransferCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesBackwardsTransferCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighChamberCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesSpecDrop;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferBackwardCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferSpecPreDrop;
import org.firstinspires.ftc.teamcode.commands.TransferSpecimenDropCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.TurretNormalResetCommand;
import org.firstinspires.ftc.teamcode.commands.TurretResetTransferCommand;
import org.firstinspires.ftc.teamcode.commands.TurretSideTransferCommand;
import org.firstinspires.ftc.teamcode.commands.groups.RetractCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightDetection;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Intake check")
public class Intakecheckclaw extends CommandOpMode {

    private DriveSubsystem m_drive1;
    private DriveSubsystem m_drive2;
    private DefaultDrive m_driveCommand;
    private DefaultDrive m_driveCommand2;
    private IntakeSubsystem     intakeSubsystem;
    private TransferSubsystem   transferSubsystem;
    private SlidesSubsystem     slidesSubsystem;
    private RobotStateSubsystem robotState;
    private double driveSpeed = 1.0;

    private GamepadEx driver;
    private AscentSubsystem ascent;
    private GamepadEx operator;


    @Override
    public void initialize() {
        // 1) Gamepads
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        ascent = new AscentSubsystem(hardwareMap);


        intakeSubsystem   = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);

        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);
        robotState = new RobotStateSubsystem();


        m_drive1 = new DriveSubsystem(hardwareMap, telemetry);
        register(m_drive1);

// Use RunCommand to merge both gamepads
        m_drive1.setDefaultCommand(new RunCommand(() -> {
            double leftY = stronger(driver.getLeftY(), operator.getLeftY());
            double leftX = stronger(driver.getLeftX(), operator.getLeftX());
            double rightX = stronger(driver.getRightX(), operator.getRightX());

            m_drive1.drive(leftX, leftY, rightX * 0.5, driveSpeed);
        }, m_drive1));


//        register(m_drive2);
//        m_drive2.setDefaultCommand(m_driveCommand2);


        //Sample Intake
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new SequentialCommandGroup(
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new IntakeClawYawBaseCommand(intakeSubsystem),
//                        new IntakeSlidesOutHalfCommand(intakeSubsystem),
//                        new IntakePivotMid(intakeSubsystem, robotState)
//                )
//        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new TransferStowCommand(transferSubsystem),
                        new InstantCommand(()->driveSpeed = 0.7),
                        new IntakeColorNeutralCommand(intakeSubsystem),
                        new FireOneShotCommand(intakeSubsystem)
//                        new IntakePivotDownCommand(intakeSubsystem, robotState)
                )

        );

        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(

                        new InstantCommand(()->intakeSubsystem.intakePivotMid()),
                        new InstantCommand(()->intakeSubsystem.intakeClawOpen())
                )

        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        new IntakePivotDownCommand(intakeSubsystem, robotState),
                        new WaitCommand(200),
                        new ClawCloseCommand(intakeSubsystem)

                )

        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(()->driveSpeed = 0.5),
                        new IntakeClawOpenCommand(intakeSubsystem),
                        new IntakeColorBlueCommand(intakeSubsystem),
                        new FireOneShotCommand(intakeSubsystem)
//                        new IntakePivotDownCommand(intakeSubsystem, robotState)
                )

        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(

                        new InstantCommand(()->driveSpeed = 1.0),
                        new IntakePivotDownCommand(intakeSubsystem, robotState),
                        new WaitCommand(300),
                        new ClawCloseCommand(intakeSubsystem),
                        new WaitCommand(100),
                        new TurretNormalResetCommand(intakeSubsystem),
                        new InstantCommand(()->intakeSubsystem.IntakePivotPos()),
                        new WaitCommand(700),
//                        new TurretResetTransferCommand(intakeSubsystem),
                        new IntakeSlidesInTransferCommand(intakeSubsystem, transferSubsystem)
//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(

//                        new InstantCommand(()->driveSpeed = 1.0),
//                        new IntakePivotDownCommand(intakeSubsystem, robotState),
//                        new WaitCommand(300),
//                        new ClawCloseCommand(intakeSubsystem),
//                        new WaitCommand(100),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new InstantCommand(()->intakeSubsystem.IntakePivotPos()),
//                        new WaitCommand(700),
////                        new TurretResetTransferCommand(intakeSubsystem),
//                        new IntakeSlidesInTransferCommand(intakeSubsystem, transferSubsystem),
//                        new InstantCommand(()->intakeSubsystem.turretSideTransfer())
//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)
                        new IntakeClawYawBaseCommand(intakeSubsystem)



                )
        );

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new SequentialCommandGroup(

                       new IntakeClawOpenCommand(intakeSubsystem)
//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                )
        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new IntakeSlidesOutCommand(intakeSubsystem)


//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                )
        ).whenReleased(
                new SequentialCommandGroup(
                        new IntakeClawOpenCommand(intakeSubsystem),
                        new WaitCommand(100),
                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)
                )
        );


//        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(()->transferSubsystem.backwardsTransfer()),
//                        new IntakePivotDownCommand(intakeSubsystem, robotState),
//                        new WaitCommand(300),
//                        new ClawCloseCommand(intakeSubsystem),
//                        new WaitCommand(700),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState),
//                        new IntakePivotUpCommand(intakeSubsystem, robotState),
//                        new WaitCommand(700),
////                        new TurretResetTransferCommand(intakeSubsystem),
//                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
//                        new InstantCommand(()->intakeSubsystem.turretSideTransfer())
////                                new ClawLooseCommand(intakeSubsystem),
//                        //add outtake pick
////                        new IntakeClawOpenCommand(intakeSubsystem),
////                        new TurretNormalResetCommand(intakeSubsystem),
////                        new WaitCommand(500),
////                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)
//
////                                new ResetArmForTransferCommand(intakeSubsystem)
//
//
//                ));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(()->intakeSubsystem.IntakePivotPos()),
                        new IntakeClawOpenCommand(intakeSubsystem),
                        new TransferBackwardCommand(transferSubsystem),
                        new WaitCommand(200),
                        new CloseGripplerCommand(transferSubsystem),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new TransferSpecPreDrop(transferSubsystem),
                                new SlidesSpecDrop(slidesSubsystem)
                        )


//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                ));

        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(


//                        new WaitCommand(300),
                        new OpenGripplerCommand(transferSubsystem),
                        new TransferBackwardCommand(transferSubsystem),
                        new SlidesBackwardsTransferCommand(slidesSubsystem)
//                        new WaitCommand(1000),
//                        new InstantCommand(()->intakeSubsystem.fireOneShot())




//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                ));

        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                                new TransferSpecimenDropCommand(transferSubsystem)

//                        new WaitCommand(300),
//                        new OpenGripplerCommand(transferSubsystem),
//                        new TransferBackwardCommand(transferSubsystem),
//                        new SlidesStowCommand(slidesSubsystem)
//                        new WaitCommand(1000),
//                        new InstantCommand(()->intakeSubsystem.fireOneShot())




//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                ));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new SequentialCommandGroup(
                        new IntakeSlidesOutCommand(intakeSubsystem)

//                        new WaitCommand(300),
//                        new OpenGripplerCommand(transferSubsystem),
//                        new TransferBackwardCommand(transferSubsystem),
//                        new SlidesStowCommand(slidesSubsystem)
//                        new WaitCommand(1000),
//                        new InstantCommand(()->intakeSubsystem.fireOneShot())




//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                ));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new IntakeClawYawSecondCommand(intakeSubsystem)
        );
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new IntakeClawYawBaseCommand(intakeSubsystem)
        );

//        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new IntakeClawYawBaseCommand(intakeSubsystem)
//        );

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new IntakeClawYawSecondCommand(intakeSubsystem)
        );

//        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
//                new RunCommand(() -> intakeSubsystem.intakeClawYawBase())
//        );

                new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whileActiveContinuous(
                        new SequentialCommandGroup(new InstantCommand(()->driveSpeed = 1.0),
                                new IntakePivotDownCommand(intakeSubsystem, robotState),
                                new WaitCommand(300),
                                new ClawCloseCommand(intakeSubsystem),
                                new WaitCommand(100),
                                new TurretNormalResetCommand(intakeSubsystem),
                                new InstantCommand(()->intakeSubsystem.IntakePivotPos()),
                                new WaitCommand(700),
//                        new TurretResetTransferCommand(intakeSubsystem),
                                new IntakeSlidesInTransferCommand(intakeSubsystem, transferSubsystem))
                );

        new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(
                        new SequentialCommandGroup(new InstantCommand(()->driveSpeed = 1.0),
                                new IntakePivotDownCommand(intakeSubsystem, robotState),
                                new WaitCommand(300),
                                new ClawCloseCommand(intakeSubsystem)
//                                new WaitCommand(100),
//                                new TurretNormalResetCommand(intakeSubsystem),
//                                new InstantCommand(()->intakeSubsystem.IntakePivotPos()),
//                                new WaitCommand(700),
////                        new TurretResetTransferCommand(intakeSubsystem),
//                                new IntakeSlidesInTransferCommand(intakeSubsystem, transferSubsystem))
                )
                );

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(
                        new RunCommand(() -> intakeSubsystem.intakeSlidesOut())
                );

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new IntakePivotMid(intakeSubsystem, robotState)
//                                new ClawLooseCommand(intakeSubsystem),
                        //add outtake pick
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

//                                new ResetArmForTransferCommand(intakeSubsystem)


                ));



//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new IntakePivotDownCommand(intakeSubsystem, robotState)
//        );


//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new SequentialCommandGroup(
//                        new IntakeClawOpenCommand(intakeSubsystem),
//                        new TurretNormalResetCommand(intakeSubsystem),
//                        new IntakeSlidesOutCommand(intakeSubsystem),
//                        new IntakeClawYawBaseCommand(intakeSubsystem),
//                        new IntakePivotMid(intakeSubsystem, robotState)
//                )
//        );

//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
//                .whileActiveContinuous(
//                        new RunCommand(() -> intakeSubsystem.adjustTurret(IntakeSubsystem.TURRET_STEP),
//                                intakeSubsystem)
//                );
//
//// Right trigger → move turret right
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
//                .whileActiveContinuous(
//                        new RunCommand(() -> intakeSubsystem.adjustTurret(-IntakeSubsystem.TURRET_STEP),
//                                intakeSubsystem)
//                );


//        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileActiveContinuous(
//                new InstantCommand(intakeSubsystem::IncrTurretRight)
//        );


//        driver.getGamepadButton(GamepadKeys.Button.X).whileActiveContinuous(
//                new IntakeClawYawSecondCommand(intakeSubsystem)
//        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
//                                new IntakePivotDownCommand(intakeSubsystem, robotState),
//                                new WaitCommand(300),
//                                new ClawCloseCommand(intakeSubsystem),
//                                new WaitCommand(200),
//                                new TurretNormalResetCommand(intakeSubsystem),
//                                new IntakePivotIntakePosCommand(intakeSubsystem, robotState),
//                                new IntakePivotUpCommand(intakeSubsystem, robotState),
//                                new WaitCommand(700),
//                                new TurretResetTransferCommand(intakeSubsystem),
//                                new IntakeSlidesInTransferCommand(intakeSubsystem, transferSubsystem),
////                                new ClawLooseCommand(intakeSubsystem),
//                                        //add outtake pick
//                                new IntakeClawOpenCommand(intakeSubsystem),
//                                new TurretNormalResetCommand(intakeSubsystem),
//                                new WaitCommand(500),
//                                new IntakePivotIntakePosCommand(intakeSubsystem, robotState)
                                  new InstantCommand(()->driveSpeed = 1.0),
                                  new RetractCommandGroup(slidesSubsystem, transferSubsystem, robotState, intakeSubsystem)
//                        new InstantCommand(()->intakeSubsystem.intakeClawLoose())







                )
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new CloseGripplerCommand(transferSubsystem),
                        //Transfer & Slides
                        new WaitCommand(200),
                        new IntakeClawOpenCommand(intakeSubsystem),
                        new InstantCommand(()-> intakeSubsystem.intakeSlidesFrontTransfer()),
                        new InstantCommand(()-> transferSubsystem.flipTransfer()),
                        new TurretNormalResetCommand(intakeSubsystem),
                        new WaitCommand(200),

                        new ParallelCommandGroup(
                                new InstantCommand(intakeSubsystem::IntakePivotPos),
                                new SlidesHighBasketCommand(slidesSubsystem)

                        ),

                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                        new InstantCommand(intakeSubsystem::intakeClawOpen)


                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(    ascent::moveBackward)
                .whenReleased(   ascent::stop);

        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(    ascent::moveForward)
                .whenReleased(   ascent::stop);

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new TransferSpecPreDrop(transferSubsystem),
//                        new CloseGripplerCommand(transferSubsystem),
                        //Transfer & Slides
//                        new WaitCommand(200),
                        new IntakeClawOpenCommand(intakeSubsystem),
                        new InstantCommand(()-> intakeSubsystem.intakeSlidesFrontTransfer()),
//                        new InstantCommand(()-> transferSubsystem.flipTransfer()),
                        new TurretNormalResetCommand(intakeSubsystem),
                        new WaitCommand(200),

//                        new ParallelCommandGroup(
//                                new InstantCommand(intakeSubsystem::IntakePivotPos),
//                                new SlidesHighBasketCommand(slidesSubsystem)
//
//                        ),
                        new InstantCommand(intakeSubsystem::IntakePivotPos),

                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)


                )
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new TransferSpecPreDrop(transferSubsystem),
//                        new CloseGripplerCommand(transferSubsystem),
                        //Transfer & Slides
//                        new WaitCommand(200),
                        new IntakeClawOpenCommand(intakeSubsystem),
                        new InstantCommand(()-> intakeSubsystem.intakeSlidesFrontTransfer()),
//                        new InstantCommand(()-> transferSubsystem.flipTransfer()),
                        new TurretNormalResetCommand(intakeSubsystem),
                        new WaitCommand(200),

//                        new ParallelCommandGroup(
//                                new InstantCommand(intakeSubsystem::IntakePivotPos),
//                                new SlidesHighBasketCommand(slidesSubsystem)
//
//                        ),
                        new InstantCommand(intakeSubsystem::IntakePivotPos),

                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)


                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new OpenGripplerCommand(transferSubsystem),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new TransferSpecPreDrop(transferSubsystem),
                                new SlidesStowCommand(slidesSubsystem)

                        )


                )
        );

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new IntakePivotMid(intakeSubsystem, robotState)


                )
        );






        //Add Color Choice Buttons
//        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//            new IntakeColorRedCommand(intakeSubsystem)
//        );
//
//        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new IntakeColorRedOrNeutralCommand(intakeSubsystem)
//        );
//
//        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
//                new IntakeColorNeutralCommand(intakeSubsystem)
//        );
//
//
//        //Drop, Reset Slides & Stow command add
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new SequentialCommandGroup(
//                        new OpenGripplerCommand(transferSubsystem),
//                        new ParallelCommandGroup(
//                                new SlidesStowCommand(slidesSubsystem),
//                                new TransferStowCommand(transferSubsystem),
//                                new IntakeResetCommand(intakeSubsystem)
//                        )
//                )
//        );
//
//        // 6) Button X: reset arm (raise pivots & resume sampling)
//        driver.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new IntakeResetCommand(intakeSubsystem));
//
//        //Specimen Pick from Larger side of submersible
//        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
//                new SequentialCommandGroup(
//                        //Reset Transfer
//                        new ParallelCommandGroup(
//                                //Add backwards specimen transfer
//                                new OpenGripplerCommand(transferSubsystem)
//                        ),
//                        //Reset Intake
//                        new IntakeResetCommand(intakeSubsystem),
//                        //Auto Aim one shot
//                        new FireOneShotCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new ClawCloseCommand(intakeSubsystem),
//                        new IntakePivotUpCommand(intakeSubsystem, robotState),
//                        //Retract for Transfer
//                        new ParallelCommandGroup(
//                                new TurretNormalResetCommand(intakeSubsystem),
//                                new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
//                                new WaitCommand(1000),
//                                new ClawLooseCommand(intakeSubsystem)
//                        )
//                )
//        );
//
//        //Drop in human player zone
//        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
//                new SequentialCommandGroup(
//                        //Reset Transfer
//                        new ParallelCommandGroup(
//                                new IntakeSlidesOutCommand(intakeSubsystem),
//                                new WaitCommand(500),
//                                new IntakeClawOpenCommand(intakeSubsystem)
//                        ),
//
//
//                        //Reset Intake
//                        new IntakeResetCommand(intakeSubsystem)
//                )
//        );
//
//        //Pick Alliance Specific Sample after dropping specimen
//
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new TransferBackwardCommand(transferSubsystem),
//                                new OpenGripplerCommand(transferSubsystem)
//                        ),
//                        //Reset Intake
//                        new IntakeResetCommand(intakeSubsystem),
//                        //Auto Aim one shot
//                        new FireOneShotCommand(intakeSubsystem),
//                        new WaitCommand(500),
//                        new ClawCloseCommand(intakeSubsystem),
//                        //Retract for Transfer
//                        new IntakePivotUpCommand(intakeSubsystem, robotState),
//                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
//                        new TurretSideTransferCommand(intakeSubsystem)
//                )
//        );
//
//        //Drop in human player zone, reset Intake and Pick from Wall
//
//        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new IntakeClawOpenCommand(intakeSubsystem),
//                                new TransferBackwardCommand(transferSubsystem)
//                        ),
//                        new WaitCommand(1000),
//                        new IntakeResetCommand(intakeSubsystem)
//                )
//
//        ).whenReleased(
//                new SequentialCommandGroup(
//                        new WaitCommand(700),
//                        new CloseGripplerCommand(transferSubsystem),
//                        //Add Slides up
//                        new SlidesHighChamberCommand(slidesSubsystem),
//                        new TransferSpecimenDropCommand(transferSubsystem)
//                )
//        );
//
//        //Drop Specimen, Intake and reset
//
//        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new OpenGripplerCommand(transferSubsystem),
//                                new FireOneShotCommand(intakeSubsystem)
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new WaitCommand(500),
//                                        new ClawCloseCommand(intakeSubsystem),
//                                        //Retract for Transfer
//                                        new IntakePivotUpCommand(intakeSubsystem, robotState),
//                                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
//                                        new TurretSideTransferCommand(intakeSubsystem)
//                                ),
//                                new TransferBackwardCommand(transferSubsystem),
//                                new SlidesStowCommand(slidesSubsystem)
//                        )
//                )
//
//        );
//
//        telemetry.addLine("A: extend/stow");
//        telemetry.addLine("B: fire one-shot");
//        telemetry.addLine("X: reset arm");
//        telemetry.update();

    }

    private double stronger(double a, double b) {
        return Math.abs(a) > Math.abs(b) ? a : b;
    }
}
