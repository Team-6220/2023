package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCmd  extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private final double setpointDegrees;
    public ArmPositionCmd(ArmSubsystem armSubsystem, double setpointDegrees){
        this.armSubsystem = armSubsystem;
        this.setpointDegrees = setpointDegrees;
        addRequirements(armSubsystem);
    }
    @Override
    public void execute() {
        double setpointPulses = setpointDegrees / ArmConstants.k_ARM_ENCODER_PCF;
        this.armSubsystem.setMotors(setpointPulses, ArmConstants.ControlType.k_POSITION);
    }
    @Override
    public boolean isFinished() {
        return (Math.abs(armSubsystem.getArmPositionDegrees()-setpointDegrees) < .5);
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}
