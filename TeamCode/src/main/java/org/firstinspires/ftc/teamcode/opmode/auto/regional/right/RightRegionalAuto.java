package org.firstinspires.ftc.teamcode.opmode.auto.regional.right;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.trajectory.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.DisplacementCommand;
import org.firstinspires.ftc.teamcode.commands.drive.trajectory.sequence.TrajectorySequenceContainerFollowCommand;
import org.firstinspires.ftc.teamcode.commands.example.ConeDropCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.StrafeRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Turn;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.TagVision;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Back;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Forward;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.Pose2dContainer;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SetReversed;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.SplineTo;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceConstraints;
import org.firstinspires.ftc.teamcode.trajectorysequence.container.TrajectorySequenceContainer;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
//438247 - Justin's Password bn

@Autonomous
public class RightRegionalAuto extends MatchOpMode {
    // Subsystems
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TagVision tagVision;
    private TurnServo turnServo;
    @Config
    public static class RightRegionalAutoConstants {

        public static Speed speed;
        public static class Speed {
            public static double baseVel = DriveConstants.MAX_VEL; // value
            public static double baseAccel = DriveConstants.MAX_ACCEL; // value
            public static double turnVel = DriveConstants.MAX_VEL; // value
            public static double turnAccel = DriveConstants.MAX_ANG_ACCEL; // value
            public static double slowVel = baseVel * 0.8; // value
            public static double slowAccel = baseAccel * 0.8; // value
            public static double slowTurnVel = turnVel * 0.8; // value
            public static double slowTurnAccel = turnAccel * 0.8; // value
            static TrajectorySequenceConstraints getPickupConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 18) {
                                return baseVel * 0.4;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getDropConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 20) {
                                return baseVel * 0.65;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getPreLoadDropConstraints() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            if (s > 48) {
                                return baseVel * 0.6;
                            } else {
                                return baseVel;
                            }

                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getParkConstraint() {
                return new TrajectorySequenceConstraints(
                        (s, a, b, c) -> {
                            return baseVel * 0.6;
                        },
                        (s, a, b, c) -> baseAccel,
                        turnVel,
                        turnAccel
                );
            }
            static TrajectorySequenceConstraints getBaseConstraints() {
                return new TrajectorySequenceConstraints(baseVel, baseAccel, turnVel, turnAccel);
            }
            static TrajectorySequenceConstraints getSlowConstraints() {
                return new TrajectorySequenceConstraints(slowVel, slowAccel, slowTurnVel, slowTurnAccel);
            }
        }

        public static Path path;
        public static class Path {
            public static PreLoad apreLoad;
            public static class PreLoad {
                public static Pose2dContainer startPose = new Pose2dContainer(34.9, -65, 90);
                public static Forward a = new Forward(34);
                public static SplineTo b = new SplineTo(28.6, -3.95, 130);
                static TrajectorySequenceContainer preload = new TrajectorySequenceContainer(Speed::getPreLoadDropConstraints, a, b);
            }


            public static Park jpark;
            public static class Park {
                public static double leftX = 7;
                public static double midX = 36;
                public static double rightX = 64;
                public static double y = -15.2;
                public static double heading = -90;
                public static double endHeading = -180;
                public enum AutoPosition {
                    lEFT,
                    MID,
                    RIGHT
                }
                public static AutoPosition autoPosition = AutoPosition.lEFT;
                static TrajectorySequenceContainer getPark(double x) {
                    switch (autoPosition) {
                        case lEFT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(4),
                                    new Turn(45),

                                    new StrafeLeft(12),
                                    new Back(4)
                            );
                        case MID:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(6),
                                    new Turn(45),

                                    new Back(3)
                            );
                        default:
                        case RIGHT:
                            return new TrajectorySequenceContainer(
                                    Speed::getParkConstraint,
                                    new Back(6),
                                    new Turn(45),

                                    new StrafeRight(10),
                                    new Back(5)
//                                    Speed::getParkConstraint,
//                                    new Back(5),
//                                    new SplineToSplineHeading(x, y, heading, endHeading)
                            );
                    }
                }
            }
        }
    }



    @Override
    public void robotInit() {
        claw = new Claw(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide( telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        tagVision = new TagVision(hardwareMap, telemetry);
        while (!isStarted() && !isStopRequested()) {
            tagVision.updateTagOfInterest();
            tagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
        double finalX = 0;
        switch (tagVision.getTag()) {
            case 1:
//                finalX = RightRegionalAutoConstants.Path.Park.leftX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.lEFT;
                break;
            case 2:
//                finalX = RightRegionalAutoConstants.Path.Park.midX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.MID;
                break;
            case 3:
//                finalX = RightRegionalAutoConstants.Path.Park.rightX;
                RightRegionalAutoConstants.Path.Park.autoPosition = RightRegionalAutoConstants.Path.Park.AutoPosition.RIGHT;
                break;
        }
        drivetrain.setPoseEstimate(RightRegionalAutoConstants.Path.PreLoad.startPose.getPose());
        PoseStorage.trajectoryPose = RightRegionalAutoConstants.Path.PreLoad.startPose.getPose();
        schedule(
                new SequentialCommandGroup(
                        /* Preload */
                        new ParallelCommandGroup(
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.PreLoad.preload)
                        ),
                        new InstantCommand(slide::slideHigh),
                        new ConeDropCommand (slide, claw),
                        new InstantCommand(slide::slideResting),
                        new WaitCommand(100),


                        new SequentialCommandGroup(//Y is this not working...
                                new TrajectorySequenceContainerFollowCommand(drivetrain, RightRegionalAutoConstants.Path.Park.getPark(finalX))
                        ),
                        new SequentialCommandGroup(
//                                new WaitCommand(1000),
//                                run(pivot::stopArm)
                        ),
                        run(() -> PoseStorage.currentPose = drivetrain.getPoseEstimate()),

                        /* Save Pose and end opmode*/

                        run(this::stop)
                )
        );
    }
}