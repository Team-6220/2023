package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.xLimiter = new SlewRateLimiter(Constants.k_TELEDRIVE_MAX_ACCELERATION);
        this.yLimiter = new SlewRateLimiter(Constants.k_TELEDRIVE_MAX_ACCELERATION);
        this.turningLimiter = new SlewRateLimiter(Constants.k_TELEDRIVE_MAX_ANGULAR_ACCELERATION);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double xTranslation = m_translationXSupplier.getAsDouble();
        double yTranslation = m_translationYSupplier.getAsDouble();
        double rotation = m_rotationSupplier.getAsDouble();
        
        // 1.5. filter inputs using sick af function
        xTranslation =  joystickInputFilter(xTranslation);
        yTranslation = joystickInputFilter(yTranslation);
        rotation = joystickInputFilter(rotation);

        // 2. Apply deadband
        xTranslation = Math.abs(xTranslation) > OIConstants.k_DEADBAND ? xTranslation : 0.0;
        yTranslation = Math.abs(yTranslation) > OIConstants.k_DEADBAND ? yTranslation : 0.0;
        rotation = Math.abs(rotation) > OIConstants.k_DEADBAND ? rotation : 0.0;

        // 3. Make the driving smoother
        // xTranslation = xLimiter.calculate(xTranslation) * Constants.k_PHYSICAL_MAX_SPEED_METERS_PER_SECOND*.25;
        // yTranslation = yLimiter.calculate(yTranslation) * Constants.k_PHYSICAL_MAX_SPEED_METERS_PER_SECOND *.25;
        // rotation = turningLimiter.calculate(rotation) * Constants.k_PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_SECOND *.25;
        xTranslation *= Constants.k_PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        yTranslation *= Constants.k_PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
        rotation *= Constants.k_PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_SECOND*.25;

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xTranslation, yTranslation, rotation,
                    m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private double joystickInputFilter(double input){
        if(input > 0){
            return (Math.pow(Math.E, OIConstants.k_JOYSTICK_INPUT_PROFILE * input) - 1)/(Math.pow(Math.E, OIConstants.k_JOYSTICK_INPUT_PROFILE) - 1);
        }
        return  ((Math.pow(Math.E, -1* OIConstants.k_JOYSTICK_INPUT_PROFILE * input) - 1) / (Math.pow(Math.E, OIConstants.k_JOYSTICK_INPUT_PROFILE) - 1)) * -1;
    }
}
