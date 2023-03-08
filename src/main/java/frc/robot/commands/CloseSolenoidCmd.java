package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class CloseSolenoidCmd extends CommandBase{
    private final IntakeSubsystem intakeSubsystem;
    public CloseSolenoidCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        if(!this.intakeSubsystem.getSolenoidState()){
            intakeSubsystem.toggleSolenoid();
        }
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
