package MP_ImprovedCommands;

import Improved_MP.MP_Subsystem;
import MotionProfiling.MPGains;
import PID_Classes.PID_Gains;
import edu.wpi.first.wpilibj.command.Command;

public class MP_Subsystem_PIDOnlyCommand extends Command{
	
	private MP_Subsystem system_;
	private MPGains gains_;
	
	private double errorSum_, lastError_, setpoint_;
	
	public MP_Subsystem_PIDOnlyCommand(MP_Subsystem system, PID_Gains pidGains) {
		requires(system);
		
		system_ = system;
		setpoint_ = system.getCurrState().pos;
	}	
	
	@Override
	protected void initialize() {
		errorSum_ = 0;
		lastError_ = 0;
	}

	@Override
	protected void execute() {
		double error = setpoint_ - system_.getPosition();	
		double output =	error * gains_.kp + errorSum_ * gains_.ki + (error - lastError_) * gains_.kd;	
		
		errorSum_ += error;
		
		if (error > 0 ^ lastError_ > 0){
			errorSum_ = 0;
		}
		
		lastError_ = error;
		
		system_.setOutput(output);
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		//time_ = Timer.getFPGATimestamp() - startingTime_;
		system_.setOutput(0);
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		end();
	}
}
