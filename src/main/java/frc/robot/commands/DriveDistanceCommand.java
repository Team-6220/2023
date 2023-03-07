package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveDistanceCommand extends CommandBase{
    private final PIDController xPID, yPID, tPID;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double[] newPos;
    private Pose2d currPose;
    private final Pose2d initPose;
    public DriveDistanceCommand(DrivetrainSubsystem drivetrainSubsystem, double[] newPos){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.newPos = newPos;
        this.xPID = new PIDController(0.4, 0, 0);
        this.yPID = new PIDController(0.2, 0, 0);
        this.tPID = new PIDController(0.2, 0, 0);
        drivetrainSubsystem.resetOdometry();
        this.currPose = drivetrainSubsystem.getPose();
        this.initPose = drivetrainSubsystem.getPose();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        currPose = drivetrainSubsystem.getPose();
        drivetrainSubsystem.drive(  
        Constants.m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
                xPID.calculate(newPos[0] - (currPose.getX()-initPose.getX())),
                0,
                0,
                drivetrainSubsystem.getGyroscopeRotation()
            )
        ));
    }

    @Override
    public boolean isFinished() {
        currPose = drivetrainSubsystem.getPose();
        return (Math.abs(newPos[0] - (currPose.getX()-initPose.getX()))<.05 && Math.abs(newPos[1] - (currPose.getY()-initPose.getY()))<.05);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(
            Constants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds (
                    0.0,
                    0.0,
                    0.0,
                    drivetrainSubsystem.getGyroscopeRotation()
                )
            )
        );
    }


}
