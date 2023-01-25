package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveDistanceCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double goal;
    private final double init_t;
    

    public DriveDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, double goal){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        init_t = m_drivetrainSubsystem.getPose().getTranslation().getX();
        this.goal = goal +init_t;
        addRequirements(m_drivetrainSubsystem);
    }
 
    @Override
    public void execute(){
        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.5,0,0,m_drivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public boolean isFinished(){
        return !(m_drivetrainSubsystem.getPose().getTranslation().getX() < goal);
    }

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    }
}
