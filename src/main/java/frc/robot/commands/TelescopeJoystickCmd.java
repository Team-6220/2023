package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeJoystickCmd extends CommandBase{
    private final TelescopeSubsystem telescopeSubsystem;
    private final Supplier<Double> jsInput;
    public TelescopeJoystickCmd(TelescopeSubsystem telescopeSubsystem, Supplier<Double> jsInput){
        this.jsInput = jsInput;
        this.telescopeSubsystem = telescopeSubsystem;
        addRequirements(telescopeSubsystem);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute(){
        double x = jsInput.get();
        x = (x>OIConstants.kDeadband)?x:0;
        telescopeSubsystem.setMotors(x);
    }

    @Override
    public void end(boolean interrupted) {
        telescopeSubsystem.setMotors(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
