package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ATWSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoATWHoldCmd extends ParallelCommandGroup{
    public AutoATWHoldCmd(DrivetrainSubsystem drivetrainSubsystem, ATWSubsystem atwSubsystem){
        addCommands(null);
    }
    
}
