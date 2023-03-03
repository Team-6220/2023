package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class PosXCmd extends CommandBase{
    private final SwerveSubsystem swerveSubsystem;
    private final double[] position;
    private final Pose2d initPose;
    private final PIDController Xpid, Ypid, Tpid;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    public PosXCmd(SwerveSubsystem swervesubsystem, double[] position){
        this.position = position;
        this.swerveSubsystem = swervesubsystem;
        this.initPose = swervesubsystem.getPose();
        this.Xpid = new PIDController(AutoConstants.kPXController, 0, 0);
        this.Ypid = new PIDController(AutoConstants.kPYController, 0, 0);
        this.Tpid = new PIDController(AutoConstants.kPThetaController, 0, 0);
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swervesubsystem);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = Xpid.calculate(swerveSubsystem.getPose().getX(), position[0]);
        double ySpeed = Ypid.calculate(swerveSubsystem.getPose().getY(), position[1]);
        double turningSpeed = Tpid.calculate(
            swerveSubsystem.getPose().getRotation().getDegrees(),
            position[2]);
            
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }

    @Override
    public boolean isFinished() {
        return ((Math.abs(swerveSubsystem.getPose().getX()-position[0])<.025)&&
        (Math.abs(swerveSubsystem.getPose().getY()-position[1])<.025)&&
        (Math.abs(swerveSubsystem.getPose().getRotation().getDegrees()-position[2])<.025));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.print("finished");
        swerveSubsystem.stopModules();
    }
}
