package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeDefaultCommand extends CommandBase{
    private final Supplier<Double> LJS, RJS;
    private final Supplier<Boolean> RBumper, LBumper;
    public IntakeDefaultCommand(Supplier<Double> LJS, Supplier<Double> RJS, Supplier<Boolean> RPressed, Supplier<Boolean> LPressed){
        this.LJS = LJS;
        this.RJS = RJS;
        this.RBumper = RPressed;
        this.LBumper = LPressed;
    }

    @Override
    public void execute(){

    }
}
