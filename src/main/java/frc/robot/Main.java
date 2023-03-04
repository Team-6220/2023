// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
    private Main() {
    }
    boolean isRecording = false;
	//autoNumber defines an easy way to change the file you are recording to/playing from, in case you want to make a
	//few different auto programs
	static final int autoNumber = 10;
	//autoFile is a global constant that keeps you from recording into a different file than the one you play from
	static final String autoFile = new String("/home/lvuser/recordedAuto" + autoNumber + ".csv");
	
	public void robotInit()
    {
		//do whatevah you do here
		
	}
	
    public void autonomous()
    {
    	//during autnomous, create new player object to read recorded file
    	BTMacroPlay player = null;
    	
    	//try to create a new player
    	//if there is a file, great - you have a new non-null object "player"
    	try 
    	{
    		 player = new BTMacroPlay();
		} 
    	
		//if not, print out an error
		catch (FileNotFoundException e)
		{
			e.printStackTrace();
		}
    	
    	//once autonomous is enabled
		while (isAutonomous())
		{
			//as long as there is a file you found, then use the player to scan the .csv file
			//and set the motor values to their specific motors
			if (player != null)
			{
				player.play(storage);
			}
			//do nothing if there is no file
		}
		
		//if there is a player and you've disabled autonomous, then flush the rest of the values
		//and stop reading the file
		if(player!= null)
		{
			player.end(storage);
		}
    	
    }
	
    public void operatorControl()
    {
		//lets make a new record object, it will feed the stuff we record into the .csv file
    	BTMacroRecord recorder = null;
        try {
			recorder = new BTMacroRecord();
		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		}
		
    	while(isOperatorControl())
    	{
    		//the statement in this "if" checks if a button you designate as your record button 
    		//has been pressed, and stores the fact that it has been pressed in a variable
    		if (storage.robot.getRecordButton().getButtonValue())
			{
    			isRecording = !isRecording;
			}  
			//if our record button has been pressed, lets start recording!
			if (isRecording)
			{
    			try
    			{
    				//if we succesfully have made the recorder object, lets start recording stuff
    				//2220 uses a storage object that we can get motors values, etc. from.
    				//if you don't need to pass an object like that in, modify the methods/classes
    				if(recorder != null)
    				{
    					recorder.record(storage);
    				}
    			
				}
				catch (IOException e) 
				{
					e.printStackTrace();
				}
			}
		}
		
		//once we're done recording, the last thing we'll do is clean up the recording using the end
		//function. more info on the end function is in the record class
    	try 
    	{
    		if(recorder != null)
    		{
    			recorder.end();
    		}
		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		}
    }
	
    public void disabled()
    {

    }
    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>
     * If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}


class BTMacroRecord {
	
	//this object writes values into the file we specify
	FileWriter writer;
	
	long startTime;
	
	public BTMacroRecord() throws IOException
	{
			//record the time we started recording
			startTime = System.currentTimeMillis();
			
			//put the filesystem location you are supposed to write to as a string 
			//as the argument in this method, as of 2015 it is /home/lvuser/recordedAuto.csv
			writer = new FileWriter(BTMain.autoFile);
	}
	

	public void record(BTStorage storage) throws IOException
	{
		if(writer != null)
		{
		//start each "frame" with the elapsed time since we started recording
		writer.append("" + (System.currentTimeMillis()-startTime));
		
		//in this chunk, use writer.append to add each type of data you want to record to the frame
		//the 2015 robot used the following motors during auto
		
		//drive motors
		writer.append("," + storage.robot.getFrontLeftMotor().get());
		writer.append("," + storage.robot.getFrontRightMotor().get());
		writer.append("," + storage.robot.getBackRightMotor().get());		
		writer.append("," + storage.robot.getBackLeftMotor().get());
		
		//barrel motors
		writer.append("," + storage.robot.getBarrelMotorLeft().get());
		writer.append("," + storage.robot.getBarrelMotorRight().get());
		
		//fork motors
		writer.append("," + storage.robot.getLeftForkLeft().get());
		writer.append("," + storage.robot.getLeftForkRight().get());
		writer.append("," + storage.robot.getRightForkLeft().get());
		writer.append("," + storage.robot.getRightForkRight().get());
		/*
		 * THE LAST ENTRY OF THINGS YOU RECORD NEEDS TO HAVE A DELIMITER CONCATENATED TO 
		 * THE STRING AT THE END. OTHERWISE GIVES NOSUCHELEMENTEXCEPTION
		 */ 
		
		//this records a true/false value from a piston
		writer.append("," + storage.robot.getToteClamp().isExtended() + "\n");
		
		/*
		 * CAREFUL. KEEP THE LAST THING YOU RECORD BETWEEN THESE TWO COMMENTS AS A
		 * REMINDER TO APPEND THE DELIMITER
		 */
		}
	}
	
	
	//this method closes the writer and makes sure that all the data you recorded makes it into the file
	public void end() throws IOException
	{
		if(writer !=null)
		{
		writer.flush();
		writer.close();
		}
	}
}

class BTMacroPlay {
	Scanner scanner;
	long startTime;

	boolean onTime = true;
	double nextDouble;
	

	public BTMacroPlay() throws FileNotFoundException
	{
		//create a scanner to read the file created during BTMacroRecord
		//scanner is able to read out the doubles recorded into recordedAuto.csv (as of 2015)
		scanner = new Scanner(new File(BTMain.autoFile));
		
		//let scanner know that the numbers are separated by a comma or a newline, as it is a .csv file
		scanner.useDelimiter(",|\\n");
		
		//lets set start time to the current time you begin autonomous
		startTime = System.currentTimeMillis();	
	}
	
	public void play(BTStorage storage)
	{
		//if recordedAuto.csv has a double to read next, then read it
		if ((scanner != null) && (scanner.hasNextDouble()))
		{
			double t_delta;
			
			//if we have waited the recorded amount of time assigned to each respective motor value,
			//then move on to the next double value
			//prevents the macro playback from getting ahead of itself and writing different
			//motor values too quickly
			if(onTime)
			{
				//take next value
				nextDouble = scanner.nextDouble();
			}
			
			//time recorded for values minus how far into replaying it we are--> if not zero, hold up
			t_delta = nextDouble - (System.currentTimeMillis()-startTime);
			
			//if we are on time, then set motor values
			if (t_delta <= 0)
			{
				//for 2015 robot. these are all the motors available to manipulate during autonomous.
				//it is extremely important to set the motors in the SAME ORDER as was recorded in BTMacroRecord
				//otherwise, motor values will be sent to the wrong motors and the robot will be unpredicatable
				storage.robot.getFrontLeftMotor().setX(scanner.nextDouble());
				storage.robot.getFrontRightMotor().setX(scanner.nextDouble());
				storage.robot.getBackRightMotor().setX(scanner.nextDouble());
				storage.robot.getBackLeftMotor().setX(scanner.nextDouble());
				
				storage.robot.getBarrelMotorLeft().setX(scanner.nextDouble());
				storage.robot.getBarrelMotorRight().setX(scanner.nextDouble());
				
				storage.robot.getLeftForkLeft().setX(scanner.nextDouble());
				storage.robot.getLeftForkRight().setX(scanner.nextDouble());
				storage.robot.getRightForkLeft().setX(scanner.nextDouble());
				storage.robot.getRightForkRight().setX(scanner.nextDouble());
				
				storage.robot.getToteClamp().set(storage.robot.getToteClamp().isExtended());
				
				//go to next double
				onTime = true;
			}
			//else don't change the values of the motors until we are "onTime"
			else
			{
				onTime = false;
			}
		}
		//end play, there are no more values to find
		else
		{
			this.end(storage);
			if (scanner != null) 
			{
				scanner.close();
				scanner = null;
			}
		}
		
	}
	
	//stop motors and end playing the recorded file
	public void end(BTStorage storage)
	{
		storage.robot.getFrontLeftMotor().setX(0);
		storage.robot.getBackLeftMotor().setX(0);
		storage.robot.getFrontRightMotor().setX(0);
		storage.robot.getBackRightMotor().setX(0);
		
		storage.robot.getBarrelMotorLeft().setX(0);
		storage.robot.getBarrelMotorRight().setX(0);
		
		storage.robot.getLeftForkLeft().setX(0);
		storage.robot.getLeftForkRight().setX(0);
		storage.robot.getRightForkLeft().setX(0);
		storage.robot.getRightForkRight().setX(0);
		//all this mess of a method does is keep the piston in the same state it ended in
		//if you want it to return to a specific point at the end of auto, change that here
		storage.robot.getToteClamp().set(storage.robot.getToteClamp().isExtended());
		
		if (scanner != null)
		{
			scanner.close();
		}
		
	}
	
}