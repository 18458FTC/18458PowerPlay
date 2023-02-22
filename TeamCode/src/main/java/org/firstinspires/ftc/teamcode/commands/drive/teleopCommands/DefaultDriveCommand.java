package org.firstinspires.ftc.teamcode.commands.drive.teleopCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    private Drivetrain drive;
    private GamepadEx driverGamepad;

    protected double multiplier;
    boolean isFieldCentric;

    public DefaultDriveCommand(Drivetrain drive, GamepadEx driverGamepad, boolean isFieldCentric) {

        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1.0;
        addRequirements(this.drive);

        this.isFieldCentric = isFieldCentric;
    }

    @Override
    public void execute() {
        if(isFieldCentric) {
            drive.fieldCentric(
                    driverGamepad.getLeftY() * multiplier,
                    driverGamepad.getLeftX() * multiplier,
                    -driverGamepad.getRightX() * multiplier
            );
        } else {
            drive.mecDrive(
                    driverGamepad.getLeftY() * multiplier, //Removed - from drivergamepad
                    driverGamepad.getLeftX() * multiplier,
                    driverGamepad.getRightX() * multiplier //Changed from -driverGamepad.getLeftY(), so the drive mturns right
            );
        }
    }



    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
