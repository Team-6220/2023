package frc.robot.commands;


import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveYCommand extends CommandBase{
    private final ProfiledPIDController yPID;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double newPos;
    private Pose2d currPose;
    private final Pose2d initPose;
    private SwerveModuleState[]states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    public DriveYCommand(DrivetrainSubsystem drivetrainSubsystem, double newPos){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.newPos = newPos;
        this.yPID = new ProfiledPIDController(1, .25, 0, new Constraints(1, 3));
        drivetrainSubsystem.resetOdometry(drivetrainSubsystem.getPose());
        this.currPose = drivetrainSubsystem.getPose();
        this.initPose = drivetrainSubsystem.getPose();
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        currPose = drivetrainSubsystem.getPose();
        double speed = yPID.calculate(currPose.getY() - initPose.getY(), newPos);
        states[0] = new SwerveModuleState(speed, new Rotation2d(0));
        states[1] = new SwerveModuleState(speed, new Rotation2d(0));
        states[2] = new SwerveModuleState(speed, new Rotation2d(0));
        states[3] = new SwerveModuleState(speed, new Rotation2d(0));
        drivetrainSubsystem.drive( 
            states
        );
    }

    @Override
    public boolean isFinished() {
        currPose = drivetrainSubsystem.getPose();
        return (Math.abs(newPos - (currPose.getY()-initPose.getY()))<.05);
    }

    @Override
    public void end(boolean interrupted) {
        states[0] = new SwerveModuleState(0, new Rotation2d(0));
        states[1] = new SwerveModuleState(0, new Rotation2d(0));
        states[2] = new SwerveModuleState(0, new Rotation2d(0));
        states[3] = new SwerveModuleState(0, new Rotation2d(0));
        drivetrainSubsystem.drive(
            states
        );
    }


}
