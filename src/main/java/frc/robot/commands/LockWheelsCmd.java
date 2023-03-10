package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LockWheelsCmd extends CommandBase{
    private final DrivetrainSubsystem driveSubsystem;
    public LockWheelsCmd(DrivetrainSubsystem drivetrainSubsystem){
        this.driveSubsystem = drivetrainSubsystem;
    }
    @Override
    public void initialize() {
        driveSubsystem.toggleLock();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
