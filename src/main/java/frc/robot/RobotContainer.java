package frc.robot;

import java.io.*;
import java.util.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ATWJoystickCmd;
import frc.robot.commands.ATWPositionCmd;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.ZeroGyroscope;
import frc.robot.commands.autos.*;
import frc.robot.commands.autos.AutoPlayerCmd;
import frc.robot.commands.autos.AutoRecorderCmd;
import frc.robot.commands.autos.PathPlannerWEventsCmd;
import frc.robot.commands.autos.SampleAutoCommand;
import frc.robot.commands.drive.CalibrateWheelsCmd;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.ResetWheelsCommand;
import frc.robot.commands.drive.SavePosCommand;
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
    private final double[] zeron = {0, 30, -30};//zero
    private final double[] flatn = {-90, 1000, -1000};//flat
    private final double[] zerop = {0, 30, -1800};
    private final double[] forty5 = {-45, 2000, -500};//notc
    private final double[] pickupn = {-115, 1300, -1400};
    private final double[] pickupp = {115, 1300, -800};//pickup
    private final double[] midn = {-50, 3922, -725};//mid cone
    private final double[] midp = {50, 3922, -1125};
    private final double[] highn = {-50, 8700, -1000};//high cone
    private final double[] highp = {50, 8700, -950};
    private final double[] stationp = {34, 750, -1038};
    private final double[] stationn = {-34, 750, -900};
    private final double[] hovern = {-95, 650, -160};
    private final double[] hoverp = {95, 650, -1760};
    private final double[] siden = {-103, 660, -160};
    private final double[] sidep = {103, 660, -1740};
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
                () -> 0d

        ));
        intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand(
            intakeSubsystem,
            m_js::getTriggerPressed,
            () -> m_js.getRawButtonPressed(2), 
            m_js2::getTrigger,
            () -> m_js2.getRawButton(2)
        ));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        try{
            new Trigger(m_controller::getRightStickButtonPressed).onTrue(new AutoRecorderCmd(swerveSubsystem,atwSubsystem,intakeSubsystem));
        }catch(IOException e){
            e.printStackTrace();
        }
        new Trigger(m_controller::getBackButtonPressed).onTrue(new SavePosCommand(swerveSubsystem));
        new Trigger(m_controller::getStartButtonPressed).onTrue(new ResetWheelsCommand(swerveSubsystem));
        //new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheels(swerveSubsystem));
        //new Trigger(m_controller::getBButtonPressed).onTrue(new UnlockWheels(swerveSubsystem));
        //zero that bih!
        new Trigger(() -> m_js2.getRawButtonPressed(7)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            zeron,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(7)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            zerop,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //flat like a door (or your mother)
        // new Trigger(() -> m_js2.getRawButtonPressed(11)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     flatn,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        //the thing that a notc would do
        // new Trigger(() -> m_js2.getRawButtonPressed(12)).onTrue(new ATWPositionCmd(
        //     atwSubsystem,
        //     forty5,
        //     () -> m_js.getThrottle(),
        //     () -> m_js2.getThrottle()
        // ));
        //pick that shi up
        new Trigger(() -> m_js2.getRawButtonPressed(3)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            pickupn,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //mid cone (negative direction)
        new Trigger(() -> m_js2.getRawButtonPressed(4)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            midn,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        //high cone (negative direction)
        new Trigger(() -> m_js2.getRawButtonPressed(5)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            highn,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(3)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            pickupp,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(4)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            midp,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(5)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            highp,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js2.getRawButtonPressed(6)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            stationn,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(6)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            stationp,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(11)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            hoverp,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js.getRawButtonPressed(12)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            sidep,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js2.getRawButtonPressed(11)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            hovern,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
        new Trigger(() -> m_js2.getRawButtonPressed(12)).onTrue(new ATWPositionCmd(
            atwSubsystem,
            siden,
            () -> m_js.getThrottle(),
            () -> m_js2.getThrottle()
        ));
   }

    public Command getAutonomousCommand() {
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("marker2", new PrintCommand("passed marker 2"));
        // return new PathPlannerWEventsCmd(swerveSubsystem, traj, eventMap);
        try{
            return new AutoPlayerCmd(swerveSubsystem,atwSubsystem,intakeSubsystem);
        }catch(IOException e){
            e.printStackTrace();
        }
        return new AutoACmd(swerveSubsystem);
    }
}
