package MotionProfiling;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class MPDriveController {

	public void setOutput(double left, double right){
		//SmartDashboard.putNumber("encoder engle: ", Robot.driveSystem.getEncodersAngle());
		//Robot.driveSystem.tank(right, left);
	}
	
	//public abstract MPDoubleSidePos getPosition();
	public abstract void reset(); 
}
