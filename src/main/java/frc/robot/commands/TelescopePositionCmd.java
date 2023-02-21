package frc.robot.commands;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopePositionCmd extends CommandBase{
    private final TelescopeSubsystem telescopeSubsystem;
    private final double setpoint;
    private final PIDController telePID;
    public TelescopePositionCmd (TelescopeSubsystem telescopeSubsystem, double position){
        this.telescopeSubsystem = telescopeSubsystem;
        this.setpoint = position;
        this.telePID = new PIDController(TelescopeConstants.kP, TelescopeConstants.kI, TelescopeConstants.kD);
        addRequirements(telescopeSubsystem);
    }
    @Override
    public void initialize() {
        super.initialize();
    }
    @Override
    public void execute() {
        telescopeSubsystem.setMotors(telePID.calculate(telescopeSubsystem.getTelescopePosition(), setpoint));
    }
    @Override
    public void end(boolean interrupted) {
        telescopeSubsystem.stopMotors();
    }
    @Override
    public boolean isFinished() {
        return (Math.abs(telescopeSubsystem.getTelescopePosition()-setpoint)<50);
    }
}
