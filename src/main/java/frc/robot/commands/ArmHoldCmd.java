package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class ArmHoldCmd extends CommandBase{
    private ArmSubsystem armSubsystem;
    private double initAngle;
    public ArmHoldCmd(ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem){
        this.initAngle = armSubsystem.getArmPositionDegrees();
        addRequirements(armSubsystem);
    }
    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
       double t = this.armSubsystem.getTelescopePosition();
       double out = .023 + .077*(t/TelescopeConstants.k_FULL_EXTENSION);
       if(this.armSubsystem.getArmPositionDegrees()>90.5){
        out *= -1;
       }else if(this.armSubsystem.getArmPositionDegrees()<=91 && this.armSubsystem.getArmPositionDegrees()>=89){
        out = 0;
       }
       this.armSubsystem.setMotors(out, ArmConstants.ControlType.k_PERCENT);
    }
}
