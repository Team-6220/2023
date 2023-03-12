package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.*;

public class AutoBalanceCommand extends CommandBase{
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final PIDController balancePIDController;
    private final Supplier<Boolean> xButton;
    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Boolean> xButton){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xButton = xButton;
        this.balancePIDController = new PIDController(.075, 0, 0);
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void execute() {
        //double armSet  = positions[0] + (this.armAdjust.get()*-5);
        double deg = drivetrainSubsystem.getGyroPitch();
            ChassisSpeeds c = new ChassisSpeeds(-balancePIDController.calculate(deg, 0),0d,0d);
            drivetrainSubsystem.drive(m_kinematics.toSwerveModuleStates(c));


         
    }
    @Override
    public boolean isFinished() {
        // return (Math.abs(atwSubsystem.getArmPositionDegrees() - positions[0])<=1.5 &&
        // (Math.abs(atwSubsystem.getTelescopePosition() - positions[1]) <= 1.5) &&
        // Math.abs(atwSubsystem.getWristPosition() - positions[2])<=90);
        return (!xButton.get()) || (Math.abs(drivetrainSubsystem.getGyroPitch())<2);
    }
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0)));
    }

}
