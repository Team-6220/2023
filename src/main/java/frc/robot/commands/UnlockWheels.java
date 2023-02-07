package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class UnlockWheels extends CommandBase{
    private final SwerveSubsystem m_DrivetrainSubsystem;

    public UnlockWheels(SwerveSubsystem drivetrainSubsystem){
        this.m_DrivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        this.m_DrivetrainSubsystem.toggleLock();
    }
    @Override
    public void end(boolean interrupted){
        
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    
}
