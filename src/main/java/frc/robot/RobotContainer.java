package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ATWJoystickCmd;
import frc.robot.commands.ATWPositionCmd;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.commands.autos.AutoACmd;
import frc.robot.commands.autos.PathPlannerWEventsCmd;
import frc.robot.commands.autos.SampleAutoCommand;
import frc.robot.commands.drive.CalibrateWheelsCmd;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.drive.UnlockWheels;
import frc.robot.subsystems.ATWSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ATWSubsystem atwSubsystem = new ATWSubsystem();
    private final XboxController m_controller = new XboxController(0);
    private final Joystick m_js = new Joystick(1);
    private final Joystick m_js2 = new Joystick(2);
    private final PathPlannerTrajectory traj;
    private final double[] position1 = {0, atwSubsystem.getTelescopePosition()};
    private final double[] position2 = {45, atwSubsystem.getTelescopePosition()};
    private final double[] position3 = {-45, atwSubsystem.getTelescopePosition()};
    private final double[] position4 = {90, atwSubsystem.getTelescopePosition()};
    private final double[] position5 = {-90, atwSubsystem.getTelescopePosition()};

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -m_controller.getRawAxis(OIConstants.kDriverYAxis),
                () -> m_controller.getRawAxis(OIConstants.kDriverXAxis),
                () -> m_controller.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !m_controller.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
        ));
        atwSubsystem.setDefaultCommand(new ATWJoystickCmd(
                atwSubsystem,
                () -> (-m_js.getY()*.5),
                () -> (m_js2.getY() * .5)
        ));
        traj = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Trigger(m_controller::getBackButtonPressed).onTrue(new ZeroGyroscope(swerveSubsystem));
        new Trigger(m_controller::getStartButtonPressed).onTrue(new CalibrateWheelsCmd(swerveSubsystem));
        new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheels(swerveSubsystem));
        new Trigger(m_controller::getBButtonPressed).onTrue(new UnlockWheels(swerveSubsystem));
        new Trigger(() -> m_js.getRawButton(2)).onTrue(new ATWPositionCmd(atwSubsystem, position1));
        new Trigger(() -> m_js.getRawButton(3)).onTrue(new ATWPositionCmd(atwSubsystem, position2));
        new Trigger(() -> m_js.getRawButton(4)).onTrue(new ATWPositionCmd(atwSubsystem, position3));
        new Trigger(() -> m_js.getRawButton(5)).onTrue(new ATWPositionCmd(atwSubsystem, position4));
        new Trigger(() -> m_js.getRawButton(6)).onTrue(new ATWPositionCmd(atwSubsystem, position5));
    }

    public Command getAutonomousCommand() {
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("marker2", new PrintCommand("passed marker 2"));
        // return new PathPlannerWEventsCmd(swerveSubsystem, traj, eventMap);
        return new AutoACmd(swerveSubsystem);
    }
}
