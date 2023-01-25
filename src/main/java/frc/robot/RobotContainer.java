// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.LockWheels;
import frc.robot.commands.SampleAutoCommand;
import frc.robot.commands.SmallAdjustmentCommand;
import frc.robot.commands.UnlockWheels;
import frc.robot.commands.ZeroGyroscope;
//import frc.robot.commands.TimedCommand;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  private final XboxController m_controller = new XboxController(0);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
            ()->(m_controller.getRawAxis(0)),
            ()->(m_controller.getRawAxis(1)*-1),
            ()->(m_controller.getRawAxis(4))
    ));

    m_armSubsystem.setDefaultCommand(new DefaultArmCommand());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Trigger(m_controller::getBackButton).onTrue(new ZeroGyroscope(m_drivetrainSubsystem));
    new Trigger(m_controller::getAButtonPressed).onTrue(new LockWheels(m_drivetrainSubsystem));
    new Trigger(m_controller::getBButtonPressed).onTrue(new UnlockWheels(m_drivetrainSubsystem));
    new Trigger(m_controller::getXButtonPressed).onTrue(new SmallAdjustmentCommand(m_drivetrainSubsystem, 1, .1));
    new Trigger(m_controller::getYButtonPressed).onTrue(new AutoClimbCommand(m_drivetrainSubsystem));
    // new Trigger(m_js1::getTriggerPressed).onTrue(new ZeroGyroscope(m_drivetrainSubsystem));
    // twoButton.onTrue(new LockWheels(m_drivetrainSubsystem));
    // threeButton.onTrue(new UnlockWheels(m_drivetrainSubsystem));
    // fourButton.onTrue(new SmallAdjustmentCommand(m_drivetrainSubsystem, 1, .1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SampleAutoCommand(m_drivetrainSubsystem);
  }

  public double joystickInputFilter(double input){
    return Math.pow(input, 3);
  }
}
