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
import frc.robot.commands.ATWPositionCmd;
import frc.robot.commands.ArmHoldCmd;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.CalibrateWheelsCmd;
import frc.robot.commands.LockWheels;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TelescopeJoystickCmd;
import frc.robot.commands.UnlockWheels;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.commands.autos.AutoACmd;
import frc.robot.commands.autos.PathPlannerWEventsCmd;
import frc.robot.commands.autos.SampleAutoCommand;
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
        traj = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
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
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("marker2", new PrintCommand("passed marker 2"));
        // return new PathPlannerWEventsCmd(swerveSubsystem, traj, eventMap);
        return new AutoACmd(swerveSubsystem);
    }
}
