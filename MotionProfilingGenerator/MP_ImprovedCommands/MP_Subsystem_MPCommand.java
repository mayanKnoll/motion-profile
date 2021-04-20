package MP_ImprovedCommands;

import Improved_MP.MP_Path;
import Improved_MP.MP_Subsystem;
import MotionProfiling.MPGains;
import PID_Classes.Setpoint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class MP_Subsystem_MPCommand extends Command{
	
	protected MP_Subsystem system_;
	
	protected MP_Path motion_;
	protected double 	startingTime_, time_;
	protected MPGains gains_;
	protected double errorSum_, lastError_;
	protected double position_, Vmax_, Vend_, acc_;
	
	public MP_Subsystem_MPCommand(MP_Subsystem system, double position, double Vmax,
			double Vend, double acc, MPGains gains) {
		
		setVariables(system, position, Vmax, Vend, acc, gains);
	}
	
	public MP_Subsystem_MPCommand(MP_Subsystem system, double position, MPGains gains){
		setVariables(system, position, system.getDefultVmax(), 0, system.getDefultAcc(), gains);
	}
	
	public MP_Subsystem_MPCommand(MP_Subsystem system, double position){
		
		setVariables(system, position, system.getDefultVmax(), 0, system.getDefultAcc(),
				system.getDefultGains());
	}
	
	protected MP_Subsystem_MPCommand(){
		
	}
	
	private void setVariables(MP_Subsystem system, double position, double Vmax,
			double Vend, double acc, MPGains gains){
		requires(system);
		
		position_ = position;
		Vmax_ = Vmax;
		Vend_ = Vend;
		acc_ = acc;
		system_ = system;
		gains_ = gains;
	}
	
	@Override
	protected void initialize() {
		Setpoint currState = system_.getCurrState();
		
		motion_ = new MP_Path(position_ - currState.pos, Vmax_, currState.vel, Vend_, acc_, acc_);
		
		startingTime_ = Timer.getFPGATimestamp();
		errorSum_ = 0;
		lastError_ = 0;
		time_ = 0;
	}

	@Override
	protected void execute() {
		time_ = Timer.getFPGATimestamp() - startingTime_;
		
		Setpoint setpoint = motion_.getCurrentState(time_);
		
		double error = setpoint.pos - system_.getPosition();
		
		double output = gains_.kv * setpoint.vel + gains_.ka * setpoint.acc +
				error * gains_.kp + errorSum_ * gains_.ki + (error - lastError_) * gains_.kd;
		
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
		return time_ >= motion_.getTotalTime();
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		//time_ = Timer.getFPGATimestamp() - startingTime_;
		system_.setCurrState(motion_.getCurrentState(time_));
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		end();
	}

}
