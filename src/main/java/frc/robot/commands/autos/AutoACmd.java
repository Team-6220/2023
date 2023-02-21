package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoACmd  extends SequentialCommandGroup{
    public AutoACmd (SwerveSubsystem swerveSubsystem){
        PathPlannerTrajectory traj1 = PathPlanner.loadPath("Path1", 4, 3); //pos x
        PathPlannerTrajectory traj2 = PathPlanner.loadPath("Path2", 2, 3); //neg y
        PathPlannerTrajectory traj3 = PathPlanner.loadPath("Path3", 4, 3); //neg x
        PathPlannerTrajectory traj4 = PathPlanner.loadPath("Path4", 4, 3); //neg y
        PathPlannerTrajectory traj5 = PathPlanner.loadPath("Path5", 4, 3); //neg y
        PathPlannerTrajectory traj6 = PathPlanner.loadPath("Path6", 4, 3); //pos x
        PathPlannerTrajectory traj7 = PathPlanner.loadPath("Path7", 4, 3); //pos y
        //auto a seq: 1,2,3,4,5,6
        addRequirements(swerveSubsystem);
        addCommands(
            new PathPlannerYCmd(swerveSubsystem, traj7)
        );
    }
    
}
