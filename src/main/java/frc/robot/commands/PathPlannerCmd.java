package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ATWSubsystem;

import java.util.*;
import java.io.*;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.*;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;

public class PathPlannerCmd extends SequentialCommandGroup {
    public PathPlannerCmd(DrivetrainSubsystem driveSubsystem,ATWSubsystem atwSubsystem,IntakeSubsystem intakeSubsystem, String pathname){
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathname, new PathConstraints(1, 2));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    double[] pos = {0d, 10d, 0d};
    double[] zeron = {0, 1, -30};//zero
    double[] pickupp = {115, 7.7, -800};//pickup
    double[] autocubehigh = {-55, 33, -639};
    //initial score
    //eventMap.put("ArmOut", new ATWAutoCmd(atwSubsystem, autocubehigh, () -> 0d, () -> 0d));
    //eventMap.put("Outtake", new ShootCubeCmd(intakeSubsystem));
    //eventMap.put("ArmZero", new ATWAutoCmd(atwSubsystem, zeron, ()->0d, ()->0d));

    //arm down
    //eventMap.put("ArmDown", new ATWAutoCmd(atwSubsystem, pickupp, () -> 0d, () -> 0d));
    eventMap.put("Lock", new LockWheelsCmd(driveSubsystem));
    //pickup intake
    //eventMap.put("Intake", new IntakeTimedCmd(intakeSubsystem));

    //arm up
    //eventMap.put("ArmUp", new ATWAutoCmd(atwSubsystem, zeron, () -> 0d, () -> 0d));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        driveSubsystem::getPose, // Pose2d supplier
        driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.m_kinematics, // SwerveDriveKinematics
        new PIDConstants(1, 0 , 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(1, 0.25, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        driveSubsystem::drive, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    addCommands(autoBuilder.fullAuto(pathGroup));
    }
}