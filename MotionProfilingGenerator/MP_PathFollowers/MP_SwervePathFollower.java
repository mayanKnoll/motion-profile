package MP_PathFollowers;

import Improved_MP.MP_DrivePath;
import Swerve_MP.MP_Swerve_DrivePath;
import Swerve_MP.SwervePoint;

public class MP_SwervePathFollower extends MP_PathFollowerCommand{

	MP_Swerve_DrivePath _path;
	
	public MP_SwervePathFollower(SwervePoint pStart, SwervePoint pEnd) {
		// TODO Auto-generated constructor stub
		_path = new MP_Swerve_DrivePath(pStart, pEnd);
	}
	
	@Override
	public MP_DrivePath getPath() {
		// TODO Auto-generated method stub
		return _path;
	}

	@Override
	protected void initialize() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected void end() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void interrupted() {
		// TODO Auto-generated method stub
		
	}

}
