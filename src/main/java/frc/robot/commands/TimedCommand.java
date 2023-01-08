package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public abstract class TimedCommand extends Command{
    public TimedCommand(double time){
        super(time);
    }
    
    @Override
    protected void execute() {
        //do nothing
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }


    @Override
    protected void interrupted() {
        throw new UnsupportedOperationException("Not supported yet.");
    }
}
