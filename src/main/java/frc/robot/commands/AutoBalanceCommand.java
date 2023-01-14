package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoBalanceCommand extends CommandBase{
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Pose3d m_pose3d;
    private final PIDController m_pid;

    public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem){
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_pose3d = new Pose3d(drivetrainSubsystem.getPose());
        this.m_pid = new PIDController(0.5,0.5,0.5);
    }



    /*
     * Balances the robot according to 
     * Preconditon: Robot must be on the charging station with wheels parallel to station sides and facing in the positive direction
     */
    @Override
    public void execute(){
        double z = m_pose3d.getZ();
        if(Math.abs(0-z)>3){
            double res = m_pid.calculate(z, 0);
            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    res,
                    0d,
                    0d)
            );
        }
    }
}