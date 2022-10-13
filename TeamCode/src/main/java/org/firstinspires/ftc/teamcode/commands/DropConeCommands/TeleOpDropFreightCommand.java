package org.firstinspires.ftc.teamcode.commands.DropConeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class TeleOpDropFreightCommand extends SequentialCommandGroup {

    public TeleOpDropFreightCommand(Drivetrain drivetrain){
        addRequirements(drivetrain);
        addCommands(
                new DriveForwardCommand(drivetrain, 4)
        );
    }

}
