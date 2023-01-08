package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class SmallAdjustmentCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public SmallAdjustmentCommand(DrivetrainSubsystem drivetrainSubsystem){
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute(){
        
    }
    
}
