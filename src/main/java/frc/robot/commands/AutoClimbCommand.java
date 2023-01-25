package frc.robot.commands;

//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoClimbCommand extends SequentialCommandGroup {
    //private double runtime;

    public AutoClimbCommand(DrivetrainSubsystem drivetrainSubsystem){
        addCommands(
            new DriveDistanceCommand(drivetrainSubsystem, 1),
            new WaitCommand(2),
            new LockWheels(drivetrainSubsystem)
        );
    }
}