package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveDistanceCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double goal;
    private final Translation2d init_t;
    

    public DriveDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, double goal){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.goal = goal;
        init_t = m_drivetrainSubsystem.getPose().getTranslation();
    }

    @Override
    public void execute(){
        Pose2d cur = m_drivetrainSubsystem.getPose();
        if(Math.abs(cur.getTranslation().getDistance(init_t)) < goal){
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0.5,0,0));
        }else{
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted){
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
    }
}
