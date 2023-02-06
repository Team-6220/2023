// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .521; 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .6477;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(288.896); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(168.746);// FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(94.920);// FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(214.536);

    public static final double k_TELEDRIVE_MAX_ACCELERATION = 3/4;
    public static final double k_TELEDRIVE_MAX_ANGULAR_ACCELERATION = 3/4;
    public static final double k_PHYSICAL_MAX_SPEED_METERS_PER_SECOND = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND/20;
    public static final double k_PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_SECOND = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND/20;

    public static final double ticksPerRevolution = 2048;
    public static final double distancePerRev = 4 * Math.PI;

    public static final SwerveDriveKinematics k_DRIVE_KINEMATICS = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public static final class AutoConstants {
        public static final double k_MAX_SPEED_METERS_PER_SECOND = Constants.k_PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double k_MAX_ANGULAR_SPEED_RADS_PER_SECOND = Constants.k_PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_SECOND / 4;
        public static final double k_MAX_ACCELERATION_METERS_PER_SECOND_SQ = 3;
        public static final double k_MAX_ANGULAR_ACCELERATION_RADS_PER_SECOND_SQ = Math.PI / 4;
        public static final double k_PX_CONTROLLER = .5;
        public static final double k_PY_CONTROLLER = .5;
        public static final double k_PTHETA_CONTROLLER = .5;
    
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        k_MAX_ANGULAR_SPEED_RADS_PER_SECOND,
                        k_MAX_ANGULAR_ACCELERATION_RADS_PER_SECOND_SQ);
    }

    public static final class OIConstants{
        public static final double k_JOYSTICK_INPUT_PROFILE = 2.5;
        public static final double k_DEADBAND = 0.0;
    }

    public static final class ArmConstants{
        //FIXME set cone high goal angle
        public static final double k_CONE_HIGH_GOAL_ANGLE_RADIANS = 0;
        //FIXME set cone mid goal angle
        public static final double k_CONE_MID_GOAL_ANGLE_RADIANS = 0;
        //FIXME set cone low goal angle
        public static final double k_CONE_LOW_GOAL_ANGLE_RADIANS = 0;
        //FIXME set cone ground pickup angle radians
        public static final double k_CONE_GROUND_UPRIGHT_PICKUP_ANGLE_RADIANS = 0;
        //FIXME set cone fallen over ground pickup angle radians
        public static final double k_CONE_GROUND_DOWN_PICKUP_ANGLE_RADIANS = 0;
        //FIXME set cone substation pickup angle
        public static final double k_CONE_SUBSTATION_PICKUP_ANGLE_RADIANS = 0;
        //FIXME set cube high goal angle
        public static final double k_CUBE_HIGH_GOAL_ANGLE_RADIANS = 0;
        //FIXME set cube mid goal angle
        public static final double k_CUBE_MID_GOAL_ANGLE_RADIANS = 0;
        //FIXME set cube low goal angle
        public static final double k_CUBE_LOW_GOAL_ANGLE_RADIANS = 0;
        //FIXME set cube ground pickup angle
        public static final double k_CUBE_GROUND_UPRIGHT_PICKUP_ANGLE_RADIANS = 0;
        //FIXME set cube substation pickup angle
        public static final double k_CUBE_SUBSTATION_PICKUP_ANGLE_RADIANS = 0;
        //FIXME set first neo can ID
        public static final int k_ARM_DRIVE_LEADER_ID = 1;
        //FIXME set second neo can ID
        public static final int k_ARM_DRIVE_FOLLOW_ID = 2;
        //FIXME set position conversion factor
        public static final double k_ARM_ENCODER_POSITION_CONVERSION_FACTOR = 1/2048;
        public static final double k_ARM_ENCODER_VELOCITY_CONVERSION_FACTOR = k_ARM_ENCODER_POSITION_CONVERSION_FACTOR/60;
        //FIXME set reversed
        public static final boolean k_MOTORS_REVERSED = false;
        //FIXME set PID constants
        public static final double k_P = .1;
        public static final double k_I = 1e-4;
        public static final double k_D = 1;
        public static final double k_Iz = 0;
        public static final double k_FF = 0;
        public static final double k_MAX_OUT = 1;
        public static final double k_MIN_OUT = -1;

    }

    public static final class TelescopeConstants{
        public static final int k_TELESCOPE_DRIVE_LEADER_ID = 0;
        public static final int k_TELESCOPE_DRIVE_FOLLOW_ID = 0;
    }

    public static final class VisionConstants{
        //FIXME set limelight angle radians
        public static final double k_LIMELIGHT_ANGLE_RADIANS = 0;
        //FIXME set limelight height from ground inches
        public static final double k_LIMELIGHT_HEIGHT_INCHES = 0;
        //FIXME set height of goal
        public static final double k_GOAL_HEIGHT_INCHES = 0;
    }

    public static final class IntakeConstants{
        //FIXME set solenoid ports
        public static final int k_LEFT_SOLENOID_PORT = 0;
        public static final int k_RIGHT_SOLENOID_PORT = 1;
        //FIXME set intake speed
        public static final double k_CUBE_INTAKE_SPEED = 0;
        public static final double k_CONE_INTAKE_SPEED = 0;
        //FIXME set limit switch input port
        public static final int k_LIMIT_SWITCH_PORT = 0;
    }
}


