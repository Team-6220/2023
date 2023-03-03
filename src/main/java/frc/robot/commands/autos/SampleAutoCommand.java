package frc.robot.commands.autos;

import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SampleAutoCommand extends SequentialCommandGroup {
    public SampleAutoCommand(SwerveSubsystem m_drivetrainSubsystem){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                    new Translation2d(1, 0),
                    new Translation2d(2, 0),
                    new Translation2d(3, 0)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4.72, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                -3, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicDriveController hController = new HolonomicDriveController(
            new PIDController(2.5, 1, 0),
            new PIDController(.5, 0, 0),
            thetaController
        );

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                m_drivetrainSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                hController,
                m_drivetrainSubsystem::setModuleStates,
                m_drivetrainSubsystem);


        addCommands(
            new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}