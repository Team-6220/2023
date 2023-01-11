package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class LockWheels extends CommandBase{
    private final DrivetrainSubsystem m_DrivetrainSubsystem;

    public LockWheels(DrivetrainSubsystem drivetrainSubsystem){
        this.m_DrivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        this.m_DrivetrainSubsystem.lockWheels();
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
