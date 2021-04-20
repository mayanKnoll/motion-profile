package MP_PathFollowers;

import Improved_MP.MP_DrivePath;
import Improved_MP.MP_Path;
import MotionProfiling.MPDriveController;
import MotionProfiling.MPGains;
import PID_Classes.Setpoint;
import Improved_MP.MP_Tank_DrivePath;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MP_TankPathFollower extends MP_PathFollowerCommand {
	
	protected MP_Path leftPath_, rightPath_;
	private MP_Tank_DrivePath path_;
	private MPGains gains_ = new MPGains();
	
	private double 	startingTime_, time_;
	
	private MPDriveController driveController_;
	
    public MP_TankPathFollower(MPDriveController driveController, MP_Tank_DrivePath path, MPGains gains) {
		// TODO Auto-generated constructor stub
    	//requires(Robot.driveSystem);
	   
    	path_ = path;
    	
	   	leftPath_ = path.getLeftPath();
	   	rightPath_ = path.getRightPath();
		
		gains_ = gains;
		driveController_ = driveController;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
		driveController_.reset();
		startingTime_ = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	time_ = Timer.getFPGATimestamp() - startingTime_;
		
			
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return time_ >= rightPath_.getTotalTime();
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
    
    public MP_DrivePath getPath(){
    	return path_;
    }
}
