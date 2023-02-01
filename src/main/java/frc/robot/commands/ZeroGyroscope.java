package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyroscope extends CommandBase{
    private final SwerveSubsystem m_swerveSubsystem;
    public ZeroGyroscope(SwerveSubsystem SwerveSubsystem){
        this.m_swerveSubsystem = SwerveSubsystem;
        addRequirements(SwerveSubsystem);
    }
    @Override
    public void initialize(){
        this.m_swerveSubsystem.zeroHeading();
    }
    @Override
    public void execute(){
        //do nothing
    }
    @Override
    public boolean isFinished(){
        return true;
    }
    @Override
    public void end(boolean interrupted){
        //do nothing
    }
}
