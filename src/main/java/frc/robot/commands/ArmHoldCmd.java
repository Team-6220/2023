package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHoldCmd extends CommandBase{
    private ArmSubsystem armSubsystem;
    private double initAngle;
    public ArmHoldCmd(ArmSubsystem armSubsystem){
        this.initAngle = armSubsystem.getArmPositionDegrees();
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
       double t = this.armSubsystem.getTelescopePosition();
       double out = .023;
       if(this.armSubsystem.getArmPositionDegrees()>0){
        out *= -1;
       }else if(this.armSubsystem.getArmPositionDegrees()<=91 && this.armSubsystem.getArmPositionDegrees()>=89){
        out = 0;
       }
       this.armSubsystem.setMotors(out, ArmConstants.ControlType.k_PERCENT);
    }
}
