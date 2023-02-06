package frc.robot;

// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 7 / 150;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 360;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.0075;
        public static final double kITurning = 0.000;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 7;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 0;
        public static final int kBackRightDriveMotorPort = 5;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 1;
        public static final int kBackRightTurningMotorPort = 6;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        public static double kFrontLeftDriveAbsoluteEncoderOffsetDeg = 0;
        public static double kBackLeftDriveAbsoluteEncoderOffsetDeg = 0;
        public static double kFrontRightDriveAbsoluteEncoderOffsetDeg = 0;
        public static double kBackRightDriveAbsoluteEncoderOffsetDeg = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 10;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 6 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
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
