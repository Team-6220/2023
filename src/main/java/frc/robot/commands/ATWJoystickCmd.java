package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;

public class ATWJoystickCmd extends CommandBase{
    private final ATWSubsystem atwSubsystem;
    private final Supplier<Double> aInput, tInput;
    public ATWJoystickCmd(ATWSubsystem atwSubsystem, Supplier<Double> aInput, Supplier<Double> tInput){
        this.atwSubsystem = atwSubsystem;
        this.aInput = aInput;
        this.tInput = tInput;
        addRequirements(atwSubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double armSpeed = aInput.get();
        double teleSpeed = tInput.get();
        this.atwSubsystem.setArmMotors(armSpeed);
        this.atwSubsystem.setTeleMotors(teleSpeed);
    }
    @Override
    public void end(boolean interrupted) {
        this.atwSubsystem.stopArm();
        this.atwSubsystem.stopTeleMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
