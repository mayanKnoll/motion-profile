package MP_ImprovedCommands;

import MP_PathFollowers.MP_SwervePathFollower;
import Swerve_MP.MP_Swerve_DrivePath;
import Swerve_MP.SwervePoint;

public class MP_SwerveAutoGenerator extends MP_SuperAutoGenerator{
	
	SwervePoint totPoint;
	
	public MP_SwerveAutoGenerator() {
		totPoint = new SwervePoint(0, 0, 0, 0, 0, 0);
	}
	
	public void addPoint(SwervePoint p) {
		list_.add(new Node(new MP_SwervePathFollower(totPoint, p), 's'));
		totPoint = p;
		System.out.println(totPoint.toString());
	}
	
}
