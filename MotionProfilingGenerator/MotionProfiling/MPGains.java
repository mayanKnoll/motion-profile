package MotionProfiling;

import PID_Classes.PID_Gains;

public class MPGains extends PID_Gains{
	public double kv;
	public double ka;
	public MPGains(){
		super();
		kv = 0;
		ka = 0;
	}
	
	public MPGains(double kv_, double ka_, double kp_, double ki_, double kd_){
		super(kp_, ki_, kd_);
		kv = kv_;
		ka = ka_;
	}
}
