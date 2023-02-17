package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCmd  extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private final double setpointDegrees;
    private final PIDController armPidController;
    public ArmPositionCmd(ArmSubsystem armSubsystem, double setpointDegrees){
        this.armSubsystem = armSubsystem;
        this.setpointDegrees = setpointDegrees;
        armPidController = new PIDController(0.005, 0, 0);
        addRequirements(armSubsystem);
    }
    @Override
    public void execute() {
        // double setpointPulses = setpointDegrees / ArmConstants.k_ARM_ENCODER_PCF;
        // this.armSubsystem.setMotors(setpointPulses, ArmConstants.ControlType.k_POSITION);
        double out = armPidController.calculate(armSubsystem.getArmPositionDegrees(), setpointDegrees);
        out = (Math.abs(out)>.75)?(out<0)?-.75:.75:out;
        this.armSubsystem.setMotors(out);
    }
    @Override
    public boolean isFinished() {
        return (Math.abs(armSubsystem.getArmPositionDegrees()-setpointDegrees) < .5);
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}
