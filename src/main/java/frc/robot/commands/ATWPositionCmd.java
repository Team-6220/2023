package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ATWSubsystem;

public class ATWPositionCmd extends CommandBase{
    private final ATWSubsystem atwSubsystem;
    private final double[] positions;
    private final PIDController armPidController, telePidController;
    public ATWPositionCmd(ATWSubsystem atwSubsystem, double[] positions){
        this.atwSubsystem = atwSubsystem;
        this.positions = positions;
        this.telePidController = new PIDController(.5, 0.25, 0);
        this.armPidController = new PIDController(0.02, 0.00, 0.00);
        addRequirements(atwSubsystem);
    }
    @Override
    public void execute() {
        double armOutput = armPidController.calculate(atwSubsystem.getArmPositionDegrees(), positions[0]);
        armOutput = (armOutput > .4)?.4:(armOutput< -.4)?-.4:armOutput;
        this.atwSubsystem.setArmMotors(armOutput);
        double telescopeOutput = telePidController.calculate(atwSubsystem.getTelescopePosition(), positions[1]);
        telescopeOutput *= (telescopeOutput > .5)?.5:(telescopeOutput<-.5)?-.5:telescopeOutput;
        this.atwSubsystem.setTeleMotors(telescopeOutput); 
    }
}
