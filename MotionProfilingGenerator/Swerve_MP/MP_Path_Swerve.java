package Swerve_MP;
import java.util.ArrayList;
import java.util.concurrent.Callable;

import PID_Classes.Setpoint;

public class MP_Path_Swerve {
	public static final double MAX_SPEED = 2;
	public static final double MAX_ACCELERATION = 1;
	
	private double distance_, V0_, Vend_;
	private double totalTime_;
	
	private boolean isLegal_ = true;
	
	public ArrayList<Segment> segments_ = new ArrayList<Segment>();
	
	public class Segment {
		public double distance, V0, Ve, totalTime, acc;
		
		public Segment(double V0, double Ve, double acc) {
			this.V0 = V0;
			this.Ve = Ve;
			this.acc = V0 < Ve? acc : -acc;
			
			this.totalTime = Math.abs((Ve - V0) / acc);
			distance = V0 * totalTime + 0.5 * this.acc * totalTime * totalTime;
		}
		
		public Segment(double V, double distance) {
			this.distance = distance;
			V0 = V;
			Ve = V;
			acc = 0;
			totalTime = distance / V;
		}
		
		public void scale(double rate) {
			distance *= rate;
			V0 *= rate;
			Ve *= rate;
			acc *= rate;
		}
		
		public Setpoint getCurrState(double time) {
			return new Setpoint(V0 * time + 0.5 * this.acc * time * time,
					V0 + this.acc * time, this.acc);
		}
		
		public double getTotalTime() {
			return totalTime;
		}
		
		public double getTotalDistance() {
			return distance;
		}
		
		public double getVe() {
			return Ve;
		}
		
		public double getV0() {
			return V0;
		}
		
		public double getAcc() {
			return acc;
		}
	}
	
	public MP_Path_Swerve(double distance){
		if (distance > 0){
			calculateTimes(distance, MAX_SPEED, 0, 0, MAX_ACCELERATION, MAX_ACCELERATION);
		}else {
			calculateTimes(distance, -MAX_SPEED, 0, 0, -MAX_ACCELERATION, MAX_ACCELERATION);
		}
	}
	
	public MP_Path_Swerve(double V0, double Vend, double acc){
		acc = Math.abs(acc);
		
		Segment temp = new Segment(V0, Vend, acc);
		
		segments_.add(temp);
		
		calculateParameters();
	}
	
	public MP_Path_Swerve(double V0, double Vend, double exelAcc, double stopAcc){
		exelAcc = Math.abs(exelAcc);
		stopAcc = Math.abs(stopAcc);
		
		if (V0 > 0 ^ Vend > 0) {
			/*segments_.add(new Segment(V0, 0, exelAcc));
			segments_.add(new Segment(0, Vend, exelAcc));*/
			
			segments_.add(new Segment(V0, Vend, exelAcc));
		} else {
			if (Math.abs(V0) < Math.abs(Vend)) {
				segments_.add(new Segment(V0, Vend, exelAcc));
			} else {
				segments_.add(new Segment(V0, Vend, stopAcc));
			}
		}
				
		V0_ = V0;
		Vend_ = Vend;
		
		
		calculateParameters();
	}
	
	public MP_Path_Swerve(double distance, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc) {
		calculateTimes(distance, Vmax, V0, Vend, exelAcc, stopAcc);
	}
	
	public MP_Path_Swerve(double distance, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc, double time) {
		calculateTimes(distance, Vmax, V0, Vend, exelAcc, stopAcc, time);
	}
	
	private void calculateTimeFliped(double distance, double Vmax, double V0, double Vend,  
			double exelAcc, double stopAcc){
		calculateTimes(-distance, -Vmax, -V0, -Vend, exelAcc, stopAcc);
		
		if (!isLegal()) {
			return;
		}
		
		scale(-1);
	}
	
	private void calculateTimes(double distance, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc) {
		calculateTimes(distance, Vmax, V0, Vend, exelAcc, stopAcc, -1);
	}
		
	private void calculateTimes(double distance, double Vmax, double V0, double Vend,
			double exelAcc, double stopAcc, double time){
		exelAcc = Math.abs(exelAcc);
		stopAcc = Math.abs(stopAcc);
		
		Vmax = Math.abs(Vmax);	
		
		V0_ = V0;
		
		if (distance == 0) {
			totalTime_ = 0;
			distance_ = 0;
			Vend_ = 0;
			V0_ = 0;
			return;
		}
		
		if (distance < 0){
			calculateTimeFliped(distance, Vmax, V0, Vend, exelAcc, stopAcc);
			return;
		}
		
		if (Vmax <= 0) {
			isLegal_ = false;
			return;
		}
		
		if (Vend < 0) {
			isLegal_ = false;
			return;
		}
		
		if (Vmax < Vend) {
			isLegal_ = false;
			return;
		}
		
		double distanceFromV0ToVend = new Segment(V0, Vend, V0 < Vend? exelAcc : stopAcc).getTotalDistance();
		if (distanceFromV0ToVend > distance) {
			
			if(time > 0) {				
				CheckVM vm_checker = new CheckVM(V0, Vend, time, distance);
				double vm = gradient_descent(vm_checker); 
				segments_.add(new Segment(V0, vm, vm_checker.getA(vm)));
				segments_.add(new Segment(vm, Vend, vm_checker.getA(vm)));
				
				calculateParameters();
				return;
			}
			
			Segment last = new Segment(V0, Vend, exelAcc);
			double lastDistance = last.getTotalDistance();
			
			// Vmin is calculated according to an equesion that we developed
			double Vmin = (-1) * Math.sqrt(exelAcc * (lastDistance - distance) + V0 * V0);
			
			segments_.add(new Segment(V0, Vmin, exelAcc));
			segments_.add(new Segment(Vmin, Vend, exelAcc));
			
			calculateParameters();
			
			return;
		}
		
		if(time > 0) {
			double a = -2;
			double b = 2 * exelAcc * time + 2 * V0 + 2 * Vend;
			double c = (-1) * (V0*V0 + Vend*Vend + 2 * exelAcc * distance);
			
			Vmax = (-b + Math.sqrt(b*b - 4*a*c)) / (2 * a);
			
			stopAcc = exelAcc;
		}
		
		if (V0 < 0) {
			Segment temp = new Segment(V0, 0, stopAcc);
			segments_.add(temp);
			distance -= temp.getTotalDistance();
			V0 = 0;
		}
		
		Segment speeding2Vmax = new Segment(V0, Vmax, exelAcc);
		Segment slowing2Vend = new Segment(Vmax, Vend, stopAcc);
		
		double speedingAndSlowingDistance = 
				speeding2Vmax.getTotalDistance() + slowing2Vend.getTotalDistance();
				
		if (speedingAndSlowingDistance > distance) {
			double newVax = Math.sqrt((2 * distance * exelAcc * stopAcc 
					+ V0 * V0 * stopAcc 
					+ Vend * Vend * exelAcc)
		
					/ (exelAcc + stopAcc));
			
			if (newVax < Vend) {
				isLegal_ = false;
				return;
			}
			
			segments_.add(new Segment(V0, newVax, exelAcc));
			segments_.add(new Segment(newVax, Vend, stopAcc));
		} else {
			segments_.add(speeding2Vmax);
			segments_.add(new Segment(Vmax, distance - speedingAndSlowingDistance));
			segments_.add(slowing2Vend);
		}
		
		calculateParameters();
	}
	
	private class CheckVM {
		double v0, ve, t, d;
		
		public CheckVM(double v0, double ve, double t, double d) {
			this.v0 = v0;
			this.ve = ve;
			this.d = d;
			this.t = t;
		}
		
		public double getA(double vm) {
			return (Math.abs(vm - v0) + Math.abs(ve - vm)) / t;
		}
		
		public double checkVm(double vm) {
			double a = getA(vm);
			double t0 = Math.abs((vm - v0) / a);
			double t1 = t - t0;
						
			return (v0 * t0) + (0.5 * (-a) * (t0*t0)) + (vm * t1) + (0.5 * a * (t1*t1)) - d;
		}
	}
	
	private static double gradient_descent(CheckVM gradient, double start, double learn_rate, int n_iter,double tolerance) {
		double vector = start;
		int i;
		for(i = 0; i < n_iter; i ++) {
			double diff = -learn_rate * gradient.checkVm(vector);
			if(Math.abs(diff) <= tolerance) {
				break;
			}
			vector += diff;
			
		}
		
		// System.out.println("num of itteratinos :" + i);
		return vector;
	}
	
	private static double gradient_descent(CheckVM gradient) {
		return gradient_descent(gradient, 0.0, 0.1, 100, 0.000001);
	}
	
	private void calcTotalTime() {
		totalTime_ = 0;
		for (int i = 0; i < segments_.size(); i ++) {
			totalTime_ += segments_.get(i).getTotalTime();
		}
	}
	
	private void calculateParameters() {
		V0_ = segments_.get(0).getV0();
		Vend_ = segments_.get(segments_.size() - 1).getVe();
		distance_ = 0;
		for (int i = 0; i < segments_.size(); i ++) {
			distance_ += segments_.get(i).getTotalDistance();
		}
		
		calcTotalTime();
	}
	
	public Setpoint getCurrentState(double currTime){
		Setpoint setpoint = new Setpoint();
		Setpoint setpointTemp;
		Segment segmentTemp = new Segment(1, 1);
		
		for (int i = 0; i < segments_.size(); i ++) {
			segmentTemp = segments_.get(i);
			if (currTime < segmentTemp.getTotalTime()) {
				setpointTemp = segmentTemp.getCurrState(currTime);
				
				setpoint.pos += setpointTemp.pos;
				setpoint.vel = setpointTemp.vel;
				setpoint.acc = segmentTemp.acc;
				
				return setpoint;
			}
			
			setpoint.pos += segmentTemp.getTotalDistance();
			currTime -= segmentTemp.getTotalTime();
		}
		
		setpoint.vel = segmentTemp.getVe();
		setpoint.acc = segmentTemp.getAcc();
		
		return setpoint;
	}
	
	public double getTotalTime(){
		return totalTime_;
	}
	
	public void scale(double scaleRate){
		for (int i = 0; i < segments_.size(); i ++) {
			segments_.get(i).scale(scaleRate);
		}
	}
	
	public boolean isLegal(){
		return isLegal_;
	}
	
	public double getV0(){
		return V0_;
	}
	
	public double getVend() {
		return Vend_;
	}
	
	public void printPath() {
		double time = 0;
		while (time < this.getTotalTime()) {
			System.out.println(this.getCurrentState(time).vel);
			time += 0.02;
		}
	}
}
