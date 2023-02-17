package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHoldCmd extends CommandBase{
    private ArmSubsystem armSubsystem;
    public ArmHoldCmd(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
       this.armSubsystem.setMotors(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
