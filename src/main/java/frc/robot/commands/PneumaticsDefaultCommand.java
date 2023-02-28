package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class PneumaticsDefaultCommand extends CommandBase{
    private final Supplier<Boolean> xButton, YButton, AButton, BButton;
    private final PneumaticSubsystem pneumaticSubsystem;
    public PneumaticsDefaultCommand(PneumaticSubsystem pneumaticSubsystem, Supplier<Boolean> xSupplier, Supplier<Boolean> ySupplier, Supplier<Boolean> aSupplier, Supplier<Boolean> bSupplier){
        this.xButton = xSupplier;
        this.AButton = aSupplier;
        this.YButton = ySupplier;
        this.BButton = bSupplier;
        this.pneumaticSubsystem = pneumaticSubsystem;
        addRequirements(pneumaticSubsystem);
    }
    @Override
    public void execute() {
        if(xButton.get()){
            pneumaticSubsystem.enableCompressor();
        }
        else if(YButton.get()){
            pneumaticSubsystem.disableCompressor();
        }
        if(AButton.get()){
            //pneumaticSubsystem.openSolenoid();
        }
        else if(BButton.get()){
            //pneumaticSubsystem.closeSolenoid();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
