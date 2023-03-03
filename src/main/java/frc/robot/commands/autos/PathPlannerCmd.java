package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PathPlannerCmd extends SequentialCommandGroup{
    public PathPlannerCmd(SwerveSubsystem swerveSubsystem, PathPlannerTrajectory traj, PIDController xpid, PIDController ypid, PIDController zpid){
        addRequirements(swerveSubsystem);
        addCommands(
            new PPSwerveControllerCommand(
            traj, 
            swerveSubsystem::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            xpid, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            ypid, // Y controller (usually the same values as X controller)
            zpid, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerveSubsystem::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem// Requires this drive subsystem
            )
        );
    }
}
