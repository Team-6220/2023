package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveDistanceCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double goal;
    private final Translation2d init_pos;
    private final double speed;


    /* Creates a command that causes the robot to drive a set distance at a constant speed. Uses an initial odometry to test distance traveled
    @param goal distance to travel in meters
    @param speed speed to drive in m/s
    */
    public DriveDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, double goal, double speed){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.goal = goal;
        this.speed = speed;
        double init_x = m_drivetrainSubsystem.getPose().getTranslation().getX();
        double init_y = m_drivetrainSubsystem.getPose().getTranslation().getY();
        init_pos = new Translation2d(init_x, init_y);
    }


    @Override
    public void execute(){
        Translation2d curPos = m_drivetrainSubsystem.getPose().getTranslation();
        if(curPos.getDistance(init_pos) < goal){
            m_drivetrainSubsystem.drive(new ChassisSpeeds(speed, 0, 0));
        }else{
            m_drivetrainSubsystem.drive(new ChassisSpeeds(0,0,0));
        }
    }

}
