package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class SmallAdjustmentCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final DoubleSupplier m_SmallAdjustment;
    private final PIDController pid;

    public SmallAdjustmentCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier smallAdjustment){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_SmallAdjustment = smallAdjustment;
        this.pid = new PIDController(0.05, 0, 0);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute(){
        double result = pid.calculate(m_drivetrainSubsystem.getPose().getX(), this.m_SmallAdjustment.getAsDouble());
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                result,
                0d,
                0d,
                m_drivetrainSubsystem.getGyroscopeRotation())
        );
    }
    

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
