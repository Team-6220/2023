package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final double absoluteEncoderOffsetDeg;

    public boolean initialized;
    private double initOffset;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset) {

        this.absoluteEncoderOffsetDeg = absoluteEncoderOffset;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        initialized = true;
        initOffset = absoluteEncoder.getAbsolutePosition();
        //driveEncoder = driveMotor.getS
        //turningEncoder = turningMotor.getEncoder();

        // driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, 0);
        turningPidController.enableContinuousInput(-180, 180);
        resetEncoders();
    }

    public double getDrivePosition() {
        //return driveEncoder.getPosition();
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        
        return (turningMotor.getSelectedSensorPosition() * 14 * Math.PI /2048 /150) % (2*Math.PI);
    }

    public double turningSensor(){
        return turningMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderRPM2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle -= absoluteEncoderOffsetDeg;
        return angle;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public void initializeWheel(){
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad()*2048/360);
    }
    public void uninitializeWheel(){
        this.initialized = false;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getState().angle.getDegrees(), state.angle.getDegrees()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }

    public void outputStates(){
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] actual state", getState().toString());
        SmartDashboard.putNumber("Swerve[" + absoluteEncoder.getDeviceID() + "] absEncoder angle", this.absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(""+ absoluteEncoder.getDeviceID() + " offset", absoluteEncoderOffsetDeg);
    }
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
