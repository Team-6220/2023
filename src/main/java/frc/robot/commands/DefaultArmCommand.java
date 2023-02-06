package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends CommandBase{
    private final ArmSubsystem m_armSubsystem;
    public DefaultArmCommand(ArmSubsystem _armSubsystem, int settingNumber){
        this.m_armSubsystem = _armSubsystem;
        addRequirements(this.m_armSubsystem);
    }
}
