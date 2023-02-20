package frc.robot.commands.autos;



import java.util.HashMap;

import javax.swing.SwingWorker;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LockWheels;
import frc.robot.subsystems.SwerveSubsystem;

public class PathPlannerWEventsCmd extends SequentialCommandGroup{
    public PathPlannerWEventsCmd(SwerveSubsystem swerveSubsystem, PathPlannerTrajectory traj, HashMap<String, Command> eventMap){
        addRequirements(swerveSubsystem);
        addCommands(
            new FollowPathWithEvents(
            new PPSwerveControllerCommand(
            traj, 
            swerveSubsystem::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(0.5, 0.05, 0.2), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0.025, 0.075, 0.3), // Y controller (usually the same values as X controller)
            new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerveSubsystem::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem// Requires this drive subsystem
            ),
            traj.getMarkers(),
            eventMap
            ),
            new LockWheels(swerveSubsystem)
        );
    }
}
