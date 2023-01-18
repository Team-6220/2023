// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 8.0;
  // Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  // Motor controllers for TalonFX. Used to access integrated encoders
  private final TalonFX m_frontLeftDrive;
  private final TalonFX m_frontRightDrive;
  private final TalonFX m_backLeftDrive;
  private final TalonFX m_backRightDrive;
  private SwerveModulePosition[] positions = new SwerveModulePosition[4];
  private final SwerveDriveOdometry odometer;
  private boolean wheelsLocked = false;
  private double previousTime;
  private SwerveModuleState[] previousStates = new SwerveModuleState[4];
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  
  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_frontLeftDrive = new TalonFX(FRONT_LEFT_MODULE_DRIVE_MOTOR);
    m_frontRightDrive = new TalonFX(FRONT_RIGHT_MODULE_DRIVE_MOTOR);
    m_backLeftDrive = new TalonFX(BACK_LEFT_MODULE_DRIVE_MOTOR);
    m_backRightDrive = new TalonFX(BACK_RIGHT_MODULE_DRIVE_MOTOR);

    m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
    this.positions[0] = new SwerveModulePosition(0, new Rotation2d(0));
    this.positions[1] = new SwerveModulePosition(0, new Rotation2d(0));
    this.positions[2] = new SwerveModulePosition(0, new Rotation2d(0));
    this.positions[3] = new SwerveModulePosition(0, new Rotation2d(0));
    this.odometer = new SwerveDriveOdometry(Constants.k_DRIVE_KINEMATICS, new Rotation2d(0), positions);
    this.previousStates[0] = new SwerveModuleState(0, new Rotation2d(0));
    this.previousStates[1] = new SwerveModuleState(0, new Rotation2d(0));
    this.previousStates[2] = new SwerveModuleState(0, new Rotation2d(0));
    this.previousStates[3] = new SwerveModuleState(0, new Rotation2d(0));

    this.previousTime = Timer.getFPGATimestamp();
    
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
//    if (m_navx.isMagnetometerCalibrated()) {
//      // We will only get valid fused headings if the magnetometer is calibrated
//      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
//    }

   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }
  public double getHeading(){
          return Math.IEEEremainder(m_navx.getAngle(), 360);
  }
  public Pose2d getPose() {
        return odometer.getPoseMeters();
  }
  public Rotation2d getRotation2d(){
          return Rotation2d.fromDegrees(getHeading());
  }
  public void stopModules(){
          m_frontLeftModule.set(0, m_frontLeftModule.getSteerAngle());
          m_frontRightModule.set(0, m_frontRightModule.getSteerAngle());
          m_backLeftModule.set(0, m_backLeftModule.getSteerAngle());
          m_backRightModule.set(0, m_backRightModule.getSteerAngle());
  }
  public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), this.positions, pose);
  }

  public void updatePositions(SwerveModuleState[] states){
        double elapsedTime = Timer.getFPGATimestamp() - this.previousTime;
        SwerveModulePosition[] previousPositions = this.positions;
        for(int i = 0; i < 4 ; i++){
             double avgVelocity = (previousStates[0].speedMetersPerSecond+states[0].speedMetersPerSecond)/2;   
             positions[i] = new SwerveModulePosition(previousPositions[0].distanceMeters + (elapsedTime * avgVelocity), states[i].angle);
        }
        
  }

  public void lockWheels(){
        //turn on parking brake
        this.wheelsLocked = true;
  }

  public void unlockWheels(){
        //turn off parking brake
        this.wheelsLocked = false;
  }

  public void setModuleStates(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states;
    if(!this.wheelsLocked){
        states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    }
    else{
        SwerveModuleState[] temp = {
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))),
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45))),
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45))),
                new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)))
        };
        states = temp;
    }
    //setModuleStates(states);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    updatePositions(states);
    odometer.update(getRotation2d(), this.positions);
  }

  //encoder access
  public double getEncoderFL(){
        double d = m_frontLeftDrive.getSelectedSensorPosition();
        double rev = d * ticksPerRevolution;
        return rev * distancePerRev;
  }
  public double getEncoderFR(){
        double d = m_frontRightDrive.getSelectedSensorPosition();
        double rev = d * ticksPerRevolution;
        return rev * distancePerRev;
  }
  public double getEncoderBL(){
        double d = m_backLeftDrive.getSelectedSensorPosition();
        double rev = d * ticksPerRevolution;
        return rev * distancePerRev;
  }
  public double getEncoderBR(){
        double d = m_backRightDrive.getSelectedSensorPosition();
        double rev = d * ticksPerRevolution;
        return rev * distancePerRev;
  }
}