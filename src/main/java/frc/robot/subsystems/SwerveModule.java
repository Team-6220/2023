package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private double absoluteEncoderOffsetDeg;

    public boolean initialized;

    private GenericEntry desiredState, init, actualState, absEncOffset, absAngle, drivePosition;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset) {

        this.absoluteEncoderOffsetDeg = absoluteEncoderOffset;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor = new TalonFX(turningMotorId);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        initialized = true;

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, 0);
        turningPidController.enableContinuousInput(-180, 180);
        
        resetEncoders();
        SwerveModuleState state = new SwerveModuleState();
        this.absEncOffset = Shuffleboard.getTab("Drivetrain").add(""+ absoluteEncoder.getDeviceID() + " turning", getTurningPosition()).getEntry();
        this.actualState = Shuffleboard.getTab("Drivetrain").add("Swerve[" + absoluteEncoder.getDeviceID() + "] actual state", getState().toString()).getEntry();
        this.desiredState = Shuffleboard.getTab("Drivetrain").add("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString()).getEntry();
        this.init = Shuffleboard.getTab("Drivetrain").add("" + absoluteEncoder.getDeviceID() +" initialized",initialized).getEntry();
        this.absAngle = Shuffleboard.getTab("Drivetrain").add(""+ absoluteEncoder.getDeviceID() + " angle", this.absoluteEncoder.getAbsolutePosition()).getEntry();
        this.drivePosition = Shuffleboard.getTab("Drivetrain").add("" + absoluteEncoder.getDeviceID() + " drive position meters", 0).getEntry();
    }

    public double getDrivePosition() {

        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter / 2048;
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

    public double getAbsoluteEncoderDeg() {
        return (absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetDeg)%360;
    }

    public void setRelative(double toSet){
        turningMotor.setSelectedSensorPosition(toSet / 14 / Math.PI * 2048 * 150);
    }
    public void updateOffset(double offset){
        this.absoluteEncoderOffsetDeg = offset;
    }
    public void updatePosition(){
        this.drivePosition.setDouble(getDrivePosition());
    }
    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderDeg()/360/7/1*150*2048);
    }
    public void uninitializeWheel(){
        this.initialized = false;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
       
        // if(!this.initialized && Math.abs(getState().angle.getDegrees())<5){
        //     this.initialized = true;
        // }
        // if(this.initialized){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getState().angle.getDegrees(), state.angle.getDegrees()));
        // }else{
        //     turningMotor.set(ControlMode.PercentOutput, initPidController.calculate(getState().angle.getDegrees(), state.angle.getDegrees()));
        // }
        this.desiredState.setString(state.toString());   
    }

    public void outputStates(){
        this.actualState.setString(getState().toString());
        this.init.setBoolean(this.initialized);
        this.absEncOffset.setDouble(getTurningPosition());
        this.absAngle.setDouble(getAbsoluteEncoderDeg());
    }
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    } 
}
