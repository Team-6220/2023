package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ATWPositionCmd;
import frc.robot.commands.ArmHoldCmd;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.CalibrateWheelsCmd;
import frc.robot.commands.LockWheels;
import frc.robot.commands.SampleAutoCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TelescopeJoystickCmd;
import frc.robot.commands.UnlockWheels;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem(telescopeSubsystem);
    private final XboxController m_controller = new XboxController(0);
    private final Joystick m_js = new Joystick(1);
    private final Joystick m_js2 = new Joystick(2);
    private final PathPlannerTrajectory traj;

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -m_controller.getRawAxis(OIConstants.kDriverYAxis),
                () -> m_controller.getRawAxis(OIConstants.kDriverXAxis),
                () -> m_controller.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !m_controller.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
        ));
        armSubsystem.setDefaultCommand(new ArmJoystickCmd(
                armSubsystem,
                () -> (-m_js.getY()*.5)
        ));
        telescopeSubsystem.setDefaultCommand(new TelescopeJoystickCmd(
                telescopeSubsystem,
                () -> -m_js2.getY()
        ));
        traj = PathPlanner.loadPath("Path", new PathConstraints(4, 3));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Trigger(m_controller::getBackButtonPressed).onTrue(new ZeroGyroscope(swerveSubsystem));
        new Trigger(m_controller::getStartButtonPressed).onTrue(new CalibrateWheelsCmd(swerveSubsystem));
        new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheels(swerveSubsystem));
        new Trigger(m_controller::getBButtonPressed).onTrue(new UnlockWheels(swerveSubsystem));
        new Trigger(m_js::getTriggerPressed).onTrue(new ArmHoldCmd(armSubsystem));
        //new Trigger(() -> m_js.getRawButtonPressed(4)).onTrue(new ATWPositionCmd(armSubsystem, telescopeSubsystem, null));
        
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
            swerveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
        }),
        new PPSwerveControllerCommand(
            traj, 
            swerveSubsystem::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(.5, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(3, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerveSubsystem::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerveSubsystem // Requires this drive subsystem
        )
    );
    }
}
