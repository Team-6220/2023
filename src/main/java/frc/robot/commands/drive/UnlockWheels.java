package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class UnlockWheels extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;

    public UnlockWheels(SwerveSubsystem swerveSubsystem){
        this.m_SwerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        this.m_SwerveSubsystem.toggleLock();
    }
    @Override
    public void end(boolean interrupted){
        
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    
}
