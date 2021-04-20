package Swerve_MP;

public class SwervePoint {
	public double x, y, z, vx, vy, vz;
	
	public SwervePoint(double x, double y, double z, double vx, double vy, double vz) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.vx = vx;
		this.vy = vy;
		this.vz = vz;
	}
	
	public SwervePoint add(SwervePoint p) {		
		return new SwervePoint(this.x + p.x, this.y + p.y, this.z + p.z, p.vx, p.vy, p.vz);
	}
	
	public SwervePoint sub(SwervePoint p) {
		return new SwervePoint(this.x - p.x, this.y - p.y, this.z - p.z, p.vx, p.vy, p.vz);
	}
	
	public String toString() {
		return x + "  " + y + "  " + z;
	}
}
