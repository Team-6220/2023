package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCubeCmd extends CommandBase{
    private int count = 0;
    private IntakeSubsystem intakeSubsystem;
    public ShootCubeCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void execute() {
        intakeSubsystem.setMotors(IntakeConstants.k_CUBE_OUTTAKE_SPEED);
        count++;
    }
    @Override
    public boolean isFinished() {
        return count >= 100;
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }
}
