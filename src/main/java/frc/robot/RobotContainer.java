package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ATWJoystickCmd;
import frc.robot.commands.ATWPositionCmd;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.commands.autos.AutoACmd;
import frc.robot.commands.drive.CalibrateWheelsCmd;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.SwerveJoystickCmd;
import frc.robot.commands.drive.UnlockWheels;
import frc.robot.subsystems.ATWSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ATWSubsystem atwSubsystem = new ATWSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final XboxController m_controller = new XboxController(0);
    private final Joystick m_js = new Joystick(1);
    private final Joystick m_js2 = new Joystick(2);
    private final double[] position1 = {0, 30, -30};//zero
    private final double[] position6 = {-90, 1000, -1000};//flat
    private final double[] position7 = {-45, 2000, -500};//notc
    private final double[] position8 = {-115, 1250, -1320};//pickup
    private final double[] position9 = {-50, 3922, -725};//mid cone
    private final double[] position10 = {-50, 8600, -725};//high cone
    // private final double[] position11 = {0,0,-1000};
    // private final double[] position12 = {0,0,-1500};

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -m_controller.getRawAxis(OIConstants.kDriverYAxis),
                () -> m_controller.getRawAxis(OIConstants.kDriverXAxis),
                () -> -m_controller.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !m_controller.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
        ));
        atwSubsystem.setDefaultCommand(new ATWJoystickCmd(
                atwSubsystem,
                () -> 0d,
                () -> (-m_js.getY()*.5),
                () -> (-m_js2.getY()*.5)

        ));
        intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(
            intakeSubsystem,
            m_js::getTrigger,
            () -> m_js.getRawButton(2), 
            m_js2::getTrigger,
            () -> m_js.getRawButton(2)
        ));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Trigger(m_controller::getBackButtonPressed).onTrue(new ZeroGyroscope(swerveSubsystem));
        new Trigger(m_controller::getStartButtonPressed).onTrue(new CalibrateWheelsCmd(swerveSubsystem));
        new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheels(swerveSubsystem));
        new Trigger(m_controller::getBButtonPressed).onTrue(new UnlockWheels(swerveSubsystem));
        //zero that bih!
        new Trigger(() -> m_js2.getRawButtonPressed(7)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            position1,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //flat like a door (or your mother)
        new Trigger(() -> m_js2.getRawButtonPressed(11)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            position6,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //the thing that a notc would do
        new Trigger(() -> m_js2.getRawButtonPressed(12)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            position7,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //pick that shi up
        new Trigger(() -> m_js2.getRawButtonPressed(3)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            position8,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //mid cone (negative direction)
        new Trigger(() -> m_js2.getRawButtonPressed(4)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            position9,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //high cone (negative direction)
        new Trigger(() -> m_js2.getRawButtonPressed(5)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            position10,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
   }

    public Command getAutonomousCommand() {
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("marker2", new PrintCommand("passed marker 2"));
        // return new PathPlannerWEventsCmd(swerveSubsystem, traj, eventMap);
        return new AutoACmd(swerveSubsystem);
    }
}
