package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmHoldCmd;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.CalibrateWheelsCmd;
import frc.robot.commands.SampleAutoCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TelescopeJoystickCmd;
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
                () -> (-m_js.getY()*.25)
        ));
        telescopeSubsystem.setDefaultCommand(new TelescopeJoystickCmd(
                telescopeSubsystem,
                () -> -m_js2.getY()
        ));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Trigger(m_controller::getBackButtonPressed).onTrue(new ZeroGyroscope(swerveSubsystem));
        new Trigger(m_controller::getStartButtonPressed).onTrue(new CalibrateWheelsCmd(swerveSubsystem));
        new Trigger(m_js::getTriggerPressed).onTrue(new ArmHoldCmd(armSubsystem));
    }

    public Command getAutonomousCommand() {
        return new SampleAutoCommand(swerveSubsystem);
    }
}
