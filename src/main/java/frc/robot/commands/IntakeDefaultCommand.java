package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends CommandBase{
    private final Supplier<Boolean> LBumper, RBumper;
    private final IntakeSubsystem intakeSubsystem;
    public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> LBumper, Supplier<Boolean> RBumper){
        this.LBumper = LBumper;
        this.RBumper = RBumper;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        if(RBumper.get()){
            intakeSubsystem.setIntakeMotors(IntakeConstants.k_CONE_INTAKE_SPEED);
        }else if(LBumper.get()){
            intakeSubsystem.setIntakeMotors(IntakeConstants.k_CUBE_INTAKE_SPEED);
        }else{
            intakeSubsystem.stopMotors();
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }
}
