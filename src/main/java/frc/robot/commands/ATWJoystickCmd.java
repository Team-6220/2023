package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;

public class ATWJoystickCmd extends CommandBase{
    private final ATWSubsystem atwSubsystem;
    private final Supplier<Double> aInput, tInput, wInput;
    public ATWJoystickCmd(ATWSubsystem atwSubsystem, Supplier<Double> aInput, Supplier<Double> tInput, Supplier<Double> wInput){
        this.atwSubsystem = atwSubsystem;
        this.aInput = aInput;
        this.tInput = tInput;
        this.wInput = wInput;
        addRequirements(atwSubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double armSpeed = aInput.get();
        double teleSpeed = tInput.get();
        double wristSpeed = wInput.get();
        //System.out.println(wristSpeed);
        this.atwSubsystem.setArmMotors(armSpeed);
        this.atwSubsystem.setTeleMotors(teleSpeed);
        this.atwSubsystem.setWristMotor(wristSpeed);
    }
    @Override
    public void end(boolean interrupted) {
        this.atwSubsystem.stopArm();
        this.atwSubsystem.stopTeleMotors();
        this.atwSubsystem.stopWristMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
