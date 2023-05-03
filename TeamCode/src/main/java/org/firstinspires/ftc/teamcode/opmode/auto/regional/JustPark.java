package org.firstinspires.ftc.teamcode.opmode.auto.regional;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Autonomous
//@Disabled
public class JustPark extends MatchOpMode {
    private int tagNum = 0;

    // Subsystems
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;


    @Override
    public void robotInit() {
        claw = new Claw( telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);

        tagVision = new TagVision(hardwareMap,  telemetry);
        while (!isStarted() && !isStopRequested())
        {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
        tagNum = tagVision.getTag();

//        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(drivetrain, 20),
                                new DriveForwardCommand(drivetrain, 12.25),
                                new StrafeLeftCommand(drivetrain, 29),
                                new DriveForwardCommand(drivetrain, 6)
                        )
                );
                return;
            }
            case 2: { //Mid
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(drivetrain, 35)
                        )
                );
                return;
            }
            case 3:
            default: { //Right
                schedule(
                        new SequentialCommandGroup(
                                new DriveForwardCommand(drivetrain, 20),
                                new DriveForwardCommand(drivetrain, 12.25),
                                new StrafeRightCommand(drivetrain, 29),
                                new DriveForwardCommand(drivetrain, 6)
                        )
                );
                return;
            }
        }
    }
}