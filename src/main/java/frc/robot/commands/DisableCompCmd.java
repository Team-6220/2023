package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DisableCompCmd extends CommandBase{
    private final IntakeSubsystem intakeSubsystem;
    public DisableCompCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {
        if(intakeSubsystem.getCompressorState()){
            intakeSubsystem.toggleCompressor();
        }
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
