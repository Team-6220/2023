package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmJoystickCmd extends CommandBase{
    Supplier<Double> m_jsInput;
    ArmSubsystem m_ArmSubsystem;
    public ArmJoystickCmd(ArmSubsystem armSubsystem, Supplier<Double> jsValue){
        this.m_ArmSubsystem = armSubsystem;
        this.m_jsInput = jsValue;
        addRequirements(armSubsystem);
    }

    @Override 
    public void initialize() {
        
    }

    @Override
    public void execute(){
        double speed = this.m_jsInput.get();
        speed = Math.abs(speed) > OIConstants.kDeadband ? speed : 0.0;
        this.m_ArmSubsystem.setMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_ArmSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
