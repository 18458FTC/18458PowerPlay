package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ConeDropCommand extends SequentialCommandGroup{
    public ConeDropCommand(Slide slide, Claw claw){
        //addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
                new InstantCommand(slide::dropSlide),
                new InstantCommand(claw::clawOpen)
        );
    }
}