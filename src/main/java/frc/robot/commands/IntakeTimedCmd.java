package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTimedCmd extends CommandBase{
    private int count;
    private final IntakeSubsystem intakeSubsystem;
    public IntakeTimedCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        count = 0;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.setMotors(IntakeConstants.k_CUBE_INTAKE_SPEED);
        count++;
    }
    @Override
    public boolean isFinished() {
        return count >= 200;
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }
}
