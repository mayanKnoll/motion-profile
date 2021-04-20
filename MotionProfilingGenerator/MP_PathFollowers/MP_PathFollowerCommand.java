package MP_PathFollowers;

import Improved_MP.MP_DrivePath;
import edu.wpi.first.wpilibj.command.Command;

public abstract class MP_PathFollowerCommand extends Command{
	
	public abstract MP_DrivePath getPath();
}
