package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;

public class ATWPositionCmd extends CommandBase{
    private final ATWSubsystem atwSubsystem;
    private final double[] positions;
    private final PIDController armPidController, telePidController, wristPidController;
    private final Supplier<Double> armAdjust, wristAdjust;
    public ATWPositionCmd(ATWSubsystem atwSubsystem, double[] positions, Supplier<Double> armAdjust, Supplier<Double> wristAdjust){
        this.atwSubsystem = atwSubsystem;
        this.positions = positions;
        //this.telePidController = new PIDController(0.00, 0.00, 0.00);
        this.telePidController = new PIDController(.002, 0, 0);
        this.armPidController = new PIDController(0.02, 0.00, 0.00);
        //this.armPidController = new PIDController(0.00, 0.00, 0.00);
        //this.wristPidController = new PIDController(0.00, 0.00, 0.00);
        this.wristPidController = new PIDController(0.004, 0.00, 0.00);
        this.armAdjust = armAdjust;
        this.wristAdjust = wristAdjust; 
        addRequirements(atwSubsystem);
    }
    @Override
    public void execute() {
        double armOutput = armPidController.calculate(atwSubsystem.getArmPositionDegrees(), positions[0]);
        armOutput = (armOutput > .3)?.3:(armOutput< -.3)?-.3:armOutput;
        this.atwSubsystem.setArmMotors(armOutput);
        double telescopeOutput = telePidController.calculate(atwSubsystem.getTelescopePosition(), positions[1]);
        telescopeOutput = (telescopeOutput > .5)?.5:(telescopeOutput<-.5)?-.5:telescopeOutput;
        telescopeOutput = (Math.abs(atwSubsystem.getTelescopePosition()-positions[1]) <= .1d)?0d:telescopeOutput;
        this.atwSubsystem.setTeleMotors(telescopeOutput);
        double wristOutput = wristPidController.calculate(atwSubsystem.getWristPosition(), positions[2]);
        wristOutput = (wristOutput > .4)?.4:(wristOutput<-.4)?-.4:wristOutput;
        this.atwSubsystem.setWristMotor(wristOutput);
         
    }
}
