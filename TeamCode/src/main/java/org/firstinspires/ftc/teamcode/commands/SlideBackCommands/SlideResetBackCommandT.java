package org.firstinspires.ftc.teamcode.commands.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetBackCommandT extends SequentialCommandGroup {
    public SlideResetBackCommandT(Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new InstantCommand(slide::slideResting, slide),
                new InstantCommand(clawMotors::moveClawIntakeFront, clawMotors),
                new InstantCommand(clawServos::orgClawPos),
                new InstantCommand(clawServos::clawOpen, clawServos)
        );
    }
}
