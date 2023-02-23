package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveDistanceCommand extends CommandBase{
    private final SwerveSubsystem m_drivetrainSubsystem;
    private final double goal;
    private final double init_t;
    

    public DriveDistanceCommand(SwerveSubsystem drivetrainSubsystem, double goal){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        init_t = m_drivetrainSubsystem.getPose().getTranslation().getX();
        this.goal = goal +init_t;
        addRequirements(m_drivetrainSubsystem);
    }
 
    @Override
    public void execute(){
        m_drivetrainSubsystem.setModuleStates(
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0.5,
                    0,
                    0,
                    m_drivetrainSubsystem.getRotation2d()
                )
            )
        );
    }

    @Override
    public boolean isFinished(){
        return !(m_drivetrainSubsystem.getPose().getTranslation().getX() < goal);
    }

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0)));
    }
}
