package PID_Classes;

public class Setpoint {
	
	public double pos;
	public double vel;
	public double acc;
	
	public Setpoint(double pos, double vel, double acc) {
		this.pos = pos;
		this.vel = vel;
		this.acc = acc;
	}
	
	public Setpoint() {
		
	}
}
