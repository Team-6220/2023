package frc.robot.commands.drive;

//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.SwerveSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup {
    //private double runtime;

    public AutoClimbCommand(SwerveSubsystem swerveSubsystem){
        addCommands(
            new DriveDistanceCommand(swerveSubsystem, 1),
            new WaitCommand(2),
            new LockWheels(swerveSubsystem)
        );
    }
}