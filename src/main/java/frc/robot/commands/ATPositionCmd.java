package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;

public class ATPositionCmd extends CommandBase{
    private final ATWSubsystem atwSubsystem;
    private final double[] positions;
    private final PIDController armPidController, telePidController, wristPidController;
    private final Supplier<Double> armAdjust, wristAdjust;
    public ATPositionCmd(ATWSubsystem atwSubsystem, double[] positions, Supplier<Double> armAdjust, Supplier<Double> wristAdjust){
        this.atwSubsystem = atwSubsystem;
        this.positions = positions;
        //this.telePidController = new PIDController(0.00, 0.00, 0.00);
        this.telePidController = new PIDController(.08, 0, 0);
        this.armPidController = new PIDController(0.05, 0.00, 0.00);
        //this.armPidController = new PIDController(0.00, 0.00, 0.00);
        //this.wristPidController = new PIDController(0.00, 0.00, 0.00);
        this.wristPidController = new PIDController(0.004, 0.00, 0.00);
        this.armAdjust = armAdjust;
        this.wristAdjust = wristAdjust; 
        addRequirements(atwSubsystem);
    }
    @Override
    public void execute() {
        double armSet  = positions[0] + (this.armAdjust.get()*-10);
        double armOutput = armPidController.calculate(atwSubsystem.getArmPositionDegrees(), armSet);
        armOutput = (armOutput > .3)?.3:(armOutput< -.3)?-.3:armOutput;
        this.atwSubsystem.setArmMotors(armOutput);

        double telescopeOutput = telePidController.calculate(atwSubsystem.getTelescopePosition(), positions[1]);
        telescopeOutput = (telescopeOutput > .3)?.3:(telescopeOutput<-.3)?-.3:telescopeOutput;
        telescopeOutput = (Math.abs(atwSubsystem.getTelescopePosition()-positions[1]) <.5)?0d:telescopeOutput;
        this.atwSubsystem.setTeleMotors(telescopeOutput);
        this.atwSubsystem.setWristMotor(0);
         
    }
    @Override
    public boolean isFinished() {
        // return (Math.abs(atwSubsystem.getArmPositionDegrees() - positions[0])<=1.5 &&
        // (Math.abs(atwSubsystem.getTelescopePosition() - positions[1]) <= 1.5) &&
        // Math.abs(atwSubsystem.getWristPosition() - positions[2])<=90);
        return false;
    }
}
