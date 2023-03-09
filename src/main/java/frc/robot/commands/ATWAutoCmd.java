package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ATWAutoCmd extends CommandBase{
    private final ATWSubsystem atwSubsystem;
    private final double[] positions;
    private final PIDController armPidController, telePidController, wristPidController;
    private final Supplier<Double> armAdjust, wristAdjust;
    public ATWAutoCmd(ATWSubsystem atwSubsystem, double[] positions, Supplier<Double> armAdjust, Supplier<Double> wristAdjust){
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
        //double armSet  = positions[0] + (this.armAdjust.get()*-5);
        double armSet = positions[0];
        double armOutput = armPidController.calculate(atwSubsystem.getArmPositionDegrees(), armSet);
        armOutput = (armOutput > .3)?.3:(armOutput< -.3)?-.3:armOutput;
        this.atwSubsystem.setArmMotors(armOutput);

        double telescopeOutput = telePidController.calculate(atwSubsystem.getTelescopePosition(), positions[1]);
        telescopeOutput = (telescopeOutput > .3)?.3:(telescopeOutput<-.3)?-.3:telescopeOutput;
        telescopeOutput = (Math.abs(atwSubsystem.getTelescopePosition()-positions[1]) <.5)?0d:telescopeOutput;
        this.atwSubsystem.setTeleMotors(telescopeOutput);

        //double wristSet = positions[2] + (this.wristAdjust.get()*-300);
        double wristSet = positions[2];
        double wristOutput = wristPidController.calculate(atwSubsystem.getWristPosition(), wristSet);
        wristOutput = (wristOutput > .4)?.4:(wristOutput<-.4)?-.4:wristOutput;
        this.atwSubsystem.setWristMotor(wristOutput);
         
    }
    @Override
    public boolean isFinished() {
        return (Math.abs(atwSubsystem.getArmPositionDegrees() - positions[0])<=1.5 &&
        (Math.abs(atwSubsystem.getTelescopePosition() - positions[1]) <= 1.5) &&
        Math.abs(atwSubsystem.getWristPosition() - positions[2])<=90);
        //return false;
    }
}
