package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ConeDropCommand;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;


    // Subsystems
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TurnServo turnServo;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        claw = new Claw(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
//        pivot.resetOffset();
    }


    @Override
    public void configureButtons() {
        /*
         *  DRIVER
         */
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, true));
        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getLeftX));

        Button recenterIMU = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));

        Button recenterIMU2 = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));

        Button slowMode = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad, true));

        /*
         * OPERATOR
         */
        //todo: Add in slide manual
        (new GamepadButton(operatorGamepad, GamepadKeys.Button.X))
                .whenPressed(new InstantCommand(slide::slideLow));
        (new GamepadButton(operatorGamepad, GamepadKeys.Button.A))
                .whenPressed(new InstantCommand(slide::slideGround));
        (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y))
                .whenPressed(new InstantCommand(slide::slideMid));
        (new GamepadButton(operatorGamepad, GamepadKeys.Button.B))
                .whenPressed(new InstantCommand(slide::slideHigh));

         (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
//                 .whenReleased()
                .whenPressed(slide::encoderRecenter);
         new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)
                 .whenPressed(new ConeDropCommand(slide, claw));
         new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
                 .whenPressed(new InstantCommand(claw::clawClose));

//        Button pivotRecenter = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
//                .whenPressed(pivot::encoderReset);
    }

    @Override
    public void matchLoop() {}
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}
