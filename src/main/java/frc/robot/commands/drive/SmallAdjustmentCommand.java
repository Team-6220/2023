package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class SmallAdjustmentCommand extends CommandBase{
    private final SwerveSubsystem m_drivetrainSubsystem;
    private final double m_SmallAdjustment;
    private final PIDController m_pid;
    private final double m_deadzone;
    private final Pose2d m_initPose;

    public SmallAdjustmentCommand(SwerveSubsystem drivetrainSubsystem, double smallAdjustment, double deadzone){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_SmallAdjustment = smallAdjustment;
        this.m_pid = new PIDController(0.5, 0, 0);
        this.m_deadzone = deadzone;
        this.m_initPose = m_drivetrainSubsystem.getPose();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute(){
        double result = m_pid.calculate(m_drivetrainSubsystem.getPose().getX() - m_initPose.getX(), this.m_SmallAdjustment);
        m_drivetrainSubsystem.setModuleStates(
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    result,
                    0d,
                    0d,
                    m_drivetrainSubsystem.getRotation2d()
                )
            )
        );
    }
    

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.setModuleStates(
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(0.0, 0.0, 0.0)
            )
        );
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(this.m_drivetrainSubsystem.getPose().getX() - this.m_initPose.getX()) < this.m_deadzone);
    }
}