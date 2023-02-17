package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopePositionCmd extends CommandBase{
    private final TelescopeSubsystem telescopeSubsystem;
    private final double setpoint;
    public TelescopePositionCmd (TelescopeSubsystem telescopeSubsystem, double position){
        this.telescopeSubsystem = telescopeSubsystem;
        this.setpoint = position;
        addRequirements(telescopeSubsystem);
    }
}
