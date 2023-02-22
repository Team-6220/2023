package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeHoldCmd extends CommandBase{
    private TelescopeSubsystem TelescopeSubsystem;
    public TelescopeHoldCmd(TelescopeSubsystem TelescopeSubsystem){
        this.TelescopeSubsystem = TelescopeSubsystem;
        addRequirements(TelescopeSubsystem);
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
       this.TelescopeSubsystem.setMotors(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
