package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroscope extends CommandBase{
    private final DrivetrainSubsystem m_DrivetrainSubsystem;
    public ZeroGyroscope(DrivetrainSubsystem DrivetrainSubsystem){
        this.m_DrivetrainSubsystem = DrivetrainSubsystem;
        addRequirements(DrivetrainSubsystem);
    }
    @Override
    public void initialize(){
        this.m_DrivetrainSubsystem.zeroGyroscope();
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
