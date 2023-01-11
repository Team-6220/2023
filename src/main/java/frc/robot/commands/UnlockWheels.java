package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class UnlockWheels extends CommandBase{
    private final DrivetrainSubsystem m_DrivetrainSubsystem;

    public UnlockWheels(DrivetrainSubsystem drivetrainSubsystem){
        this.m_DrivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        this.m_DrivetrainSubsystem.unlockWheels();
    }
    @Override
    public void end(boolean interrupted){
        
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    
}
