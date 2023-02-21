package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoACmd  extends SequentialCommandGroup{
    public AutoACmd (SwerveSubsystem swerveSubsystem){
        PathPlannerTrajectory traj1 = PathPlanner.loadPath("Path1", 4, 3);
        PathPlannerTrajectory traj2 = PathPlanner.loadPath("Path2", 2, 3);
        PathPlannerTrajectory traj3 = PathPlanner.loadPath("Path3", 4, 3);
        PathPlannerTrajectory traj4 = PathPlanner.loadPath("Path4", 4, 3);
        PathPlannerTrajectory traj5 = PathPlanner.loadPath("Path5", 4, 3);
        PathPlannerTrajectory traj6 = PathPlanner.loadPath("Path6", 4, 3);
        PathPlannerTrajectory traj7 = PathPlanner.loadPath("Path7", 4,3);
        addRequirements(swerveSubsystem);
        addCommands(
            //new PathPlannerYCmd(swerveSubsystem, traj3),
            //new PathPlannerXCmd(swerveSubsystem, traj1),
            //new PathPlannerXCmd(swerveSubsystem, traj2),
            new PathPlannerYCmd(swerveSubsystem, traj7)
        );
    }
    
}
