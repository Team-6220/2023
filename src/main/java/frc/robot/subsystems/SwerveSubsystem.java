package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
public class SwerveSubsystem extends SubsystemBase {
    private boolean lock = false;
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private SwerveModuleState[] prev = new SwerveModuleState[4];

    private SwerveModulePosition[] positions = {
        new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getState().angle.getRadians())),
        new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getState().angle.getRadians())),
        new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getState().angle.getRadians())),
        new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getState().angle.getRadians()))
    };

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    new Rotation2d(0), this.positions);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        double[] offsets = offsets();
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg += offsets[0];
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg += offsets[1];
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg += offsets[2];
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg += offsets[3];
        updateOffsets();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void unInitializeWheels(){
        frontLeft.uninitializeWheel();
        frontRight.uninitializeWheel();
        backLeft.uninitializeWheel();
        backRight.uninitializeWheel();
    }
    public boolean wheelsInitialized(){
        return (frontLeft.initialized && frontRight.initialized && backLeft.initialized && backRight.initialized);
    }
    public void resetEncoders(){
        backLeft.resetEncoders();
        backRight.resetEncoders();
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
    }
    public double[] offsets(){
        double[] arr = {frontLeft.getAbsoluteEncoderDeg(), backLeft.getAbsoluteEncoderDeg(), frontRight.getAbsoluteEncoderDeg(), backRight.getAbsoluteEncoderDeg()};
        return arr;
    }
    public void updateOffsets(){
        frontLeft.updateOffset(DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg);
        frontRight.updateOffset(DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);
        backLeft.updateOffset(DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);
        backRight.updateOffset(DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), this.positions, pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), this.positions);
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        backLeft.outputStates();
        backRight.outputStates();
        frontLeft.outputStates();
        frontRight.outputStates();
        //SmartDashboard.putNumber("yurt" , (backLeft.turningSensor()));
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
        System.out.println("set module states subsystem: " + desiredStates[0].angle.getDegrees());
        setModulePositions();
    }

    public void setModulePositions(){
        this.positions[0] = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getState().angle.getRadians()));
        this.positions[1] = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getState().angle.getRadians()));
        this.positions[2] = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getState().angle.getRadians()));
        this.positions[3] = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getState().angle.getRadians()));
    }
    public void toggleLock(){
        lock = !lock;
        if(!lock){
            setModuleStates(prev);
        }else{
            prevLock();
            SwerveModuleState[] temp = {
                    new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))),
                    new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45))),
                    new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45))),
                    new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)))   
            };
            setModuleStates(temp);
        }
    }

    public void prevLock(){
         prev[0] = new SwerveModuleState(frontLeft.getDriveVelocity(), new Rotation2d(Math.toDegrees(frontLeft.getAbsoluteEncoderDeg())));
         prev[1] = new SwerveModuleState(frontRight.getDriveVelocity(), new Rotation2d(Math.toDegrees(frontRight.getAbsoluteEncoderDeg())));
         prev[2] = new SwerveModuleState(backLeft.getDriveVelocity(), new Rotation2d(Math.toDegrees(backLeft.getAbsoluteEncoderDeg())));
         prev[3] = new SwerveModuleState(backRight.getDriveVelocity(), new Rotation2d(Math.toDegrees(backRight.getAbsoluteEncoderDeg())));
    }
}
