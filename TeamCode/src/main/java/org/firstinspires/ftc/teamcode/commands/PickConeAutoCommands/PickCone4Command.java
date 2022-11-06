package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickCone4Command extends SequentialCommandGroup{
    public PickCone4Command(Slide slide, ClawServos clawServos, ClawMotors clawMotors, Drivetrain drivetrain){
        addCommands(
                new InstantCommand(slide::slideCone4),
                new InstantCommand(clawMotors::moveIntakeF),
                new SlowDriveForwardCommand(drivetrain, 5),
                new InstantCommand(clawServos::clawClose),
                new WaitCommand(50),
                new InstantCommand(slide:: slideLow)
        );
    }
}