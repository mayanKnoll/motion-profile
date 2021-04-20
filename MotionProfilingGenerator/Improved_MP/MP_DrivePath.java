package Improved_MP;

public abstract class MP_DrivePath {
	public abstract double getY(double time);
	public abstract double getX(double time);
	public abstract double getAngleRadians(double time);
	public abstract double getAngleSimulator(double time);
	
	public abstract double getAbsulutVend();
	
	public double getAngleDegrees(double time) {
		return Math.toDegrees(getAngleRadians(time));
	}
	
	public abstract double getTotalTime();
	public abstract String toString();
}
