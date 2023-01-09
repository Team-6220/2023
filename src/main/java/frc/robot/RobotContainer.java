// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.*;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
/*import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Trigger;*/
import frc.robot.commands.DefaultDriveCommand;
//import frc.robot.commands.TimedCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.AutoConstants;

//comands import
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.ZeroGyroCommand;
import frc.robot.commands.LockWheelsCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final CommandXboxController m_controller = new CommandXboxController(0);

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
            ()->(m_controller.getRawAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            ()->(m_controller.getRawAxis(1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
            ()->(m_controller.getRawAxis(4) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
    ));

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
    Trigger back  = m_controller.back();
    m_controller.back().onTrue(new ZeroGyroCommand(m_drivetrainSubsystem));
    Trigger a = m_controller.a();
    m_controller.back().onTrue(new LockWheelsCommand(m_drivetrainSubsystem));
    Trigger x = m_controller.x();
    m_controller.back().onTrue(new AutoClimbCommand(m_drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
