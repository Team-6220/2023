package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class TimedCommand extends Command{
    public TimedCommand(double time){
        //super(time);
    }
    
    @Override
    public void execute() {
        //do nothing
    }

    @Override
    public boolean isFinished() {
        return isTimedOut();
    }

    private boolean isTimedOut(){
        if()
    }
    @Override
    public void interrupted() {
        throw new UnsupportedOperationException("Not supported yet.");
    }
}
