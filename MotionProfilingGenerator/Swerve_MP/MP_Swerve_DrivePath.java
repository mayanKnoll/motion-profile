package Swerve_MP;

import java.text.DecimalFormat;

import Improved_MP.MP_DrivePath;

public class MP_Swerve_DrivePath extends MP_DrivePath{
	
	public MP_Path_Swerve x_path, y_path, z_path;
	double Vmax = 2, exelAcc = 2, stopAcc = 2;
	double VmaxAngle = 90, AccAngle = 180;
	double totalTime = 0;

	
	public MP_Swerve_DrivePath(SwervePoint start, SwervePoint end) {
		
		x_path = new MP_Path_Swerve(end.x - start.x, Vmax, start.vx, end.vx, exelAcc, stopAcc);
		y_path = new MP_Path_Swerve(end.y - start.y, Vmax, start.vy, end.vy, exelAcc, stopAcc);
		z_path = new MP_Path_Swerve(end.z - start.z, VmaxAngle, start.vz, end.vz, AccAngle, AccAngle);

		double x_time = x_path.getTotalTime();
		double y_time = y_path.getTotalTime();
		double z_time = z_path.getTotalTime();
		
		System.out.println(x_path.getTotalTime());
		System.out.println(y_path.getTotalTime());
		System.out.println(z_path.getTotalTime());
		
		if (x_time > Math.max(y_time, z_time)) {
			totalTime = x_time;
			System.out.println("X time : " + x_time);
			y_path = new MP_Path_Swerve(end.y - start.y, Vmax, start.vy, end.vy, exelAcc, stopAcc, x_time);
			z_path = new MP_Path_Swerve(end.z - start.z, VmaxAngle, start.vz, end.vz, AccAngle, AccAngle, x_time);
		} else if (y_time > z_time) {
			totalTime = y_time;
			System.out.println("Y time : " + totalTime);
			x_path = new MP_Path_Swerve(end.x - start.x, Vmax, start.vx, end.vx, exelAcc, stopAcc, y_time);
			z_path = new MP_Path_Swerve(end.z - start.z, VmaxAngle, start.vz, end.vz, AccAngle, AccAngle, y_time);
		} else {
			totalTime = z_time;
			x_path = new MP_Path_Swerve(end.x - start.x, Vmax, start.vx, end.vx, exelAcc, stopAcc, z_time);
			y_path = new MP_Path_Swerve(end.y - start.y, Vmax, start.vy, end.vy, exelAcc, stopAcc, z_time);
		}
		
		System.out.println(x_path.getTotalTime());
		System.out.println(y_path.getTotalTime());
		System.out.println(z_path.getTotalTime());
	}
	
	@Override
	public double getY(double time) {
		// TODO Auto-generated method stub
		return y_path.getCurrentState(time).pos;
	}

	@Override
	public double getX(double time) {
		// TODO Auto-generated method stub
		return x_path.getCurrentState(time).pos;
	}

	@Override
	public double getAngleDegrees(double time) {
		return z_path.getCurrentState(time).pos;
	}
	
	public SwervePoint getCurrSwervePoint(double time) {
		return new SwervePoint(getX(time), getY(time), getAngleDegrees(time), x_path.getCurrentState(time).vel,
				y_path.getCurrentState(time).vel, z_path.getCurrentState(time).vel);
	}
	
	@Override
	public double getAngleRadians(double time) {
		// TODO Auto-generated method stub
		return Math.toRadians(getAngleDegrees(time));
	}

	@Override
	public double getAngleSimulator(double time) {
		// TODO Auto-generated method stub
		return getAngleRadians(time);//getAngleDegrees(time);
	}

	@Override
	public double getTotalTime() {
		// TODO Auto-generated method stub
		return totalTime;
	}

	@Override
	public String toString() {
		DecimalFormat f = new DecimalFormat("##.##");
		
		return "X : " + f.format(x_path.getCurrentState(totalTime).pos) + "   " + 
				"Y : " + f.format(y_path.getCurrentState(totalTime).pos) + "   " + 
				"Z : "+ f.format(z_path.getCurrentState(totalTime).pos);
	}

	@Override
	public double getAbsulutVend() {
		return Math.sqrt(Math.pow(x_path.getCurrentState(totalTime).vel, 2) + Math.pow(x_path.getCurrentState(totalTime).vel, 2));
	}

}
