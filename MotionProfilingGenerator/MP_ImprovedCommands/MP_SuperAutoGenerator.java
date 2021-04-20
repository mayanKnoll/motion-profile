package MP_ImprovedCommands;

import java.util.ArrayList;

import Improved_MP.MP_DrivePath;
import MP_PathFollowers.MP_PathFollowerCommand;
import MotionProfiling.MPDriveController;
import MotionProfiling.MPGains;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class MP_SuperAutoGenerator extends CommandGroup{

	public MPDriveController dc_;
	public MPGains gains_;
	protected ArrayList<Node> list_ = new ArrayList<Node>();
	
	protected class Node{
		public Command command;
		public char type;
		public double timeout;
		
		public Node(Command com, char type) {
			this.type = type;
			this.command = com;
			this.timeout = 0;
		}
		
		public Node(Command com, char type, double timeout) {
			this.type = type;
			this.command = com;
			this.timeout = timeout;
		}
	}
	
	/**
	 * returns the index of the last drive command.	
	 * @return
	 * the index of the last drive command.
	 * if there is no such command, returns (-1).
	 */
	protected int getLastDriveCommand(){
		for (int i = list_.size() - 1; i >= 0; i --){
			if (list_.get(i).command instanceof MP_PathFollowerCommand){
				return i;
			}
		}
		return -1;
	}
	
	public void addSeqCommand(Command command, double timeout) throws Exception{
		int lastMoveIndex = getLastDriveCommand();
		if (!(command instanceof MP_PathFollowerCommand) && lastMoveIndex >= 0){
			MP_PathFollowerCommand temp= (MP_PathFollowerCommand)(list_.get(lastMoveIndex).command);
			if (temp.getPath().getAbsulutVend() != 0){
				String errMes = "Cant add sequentioal command if robot is not in a stop\n";
				throw new Exception(errMes);
			}
		}
		
		list_.add(new Node(command, 's', timeout));
	}
	
	public void addSeqCommand(Command command) throws Exception{
		addSeqCommand(command, 0);
	}
	
	public void addParlerCommand(Command command, double timeout){
		list_.add(new Node(command, 'p', timeout));
	}
	
	public void addParlerCommand(Command command){
		addParlerCommand(command, 0);
	}
	
	public void addPerller2Previus(Command command, double timeout) throws Exception{
		int i;
		for (i = list_.size() - 1; i >= 0; i--){
			if (list_.get(i).type == 's'){
				break;
				// i is now the index of the last sequential command
			}
		}
		
		if (i < 0){
			String errMes = "Cant add perller to previus command becouse there is no such command\n";
			throw new Exception(errMes);
		}
		
		// adding the perlar command to be before the last sequential command
		list_.add(i, new Node(command, 'p', timeout));
	}
	
	public void addPerller2Previus(Command command) throws Exception{
		addPerller2Previus(command, 0);
	}
	
	/**
	 *  generates auto by addning all the commands in seq and parl.
	 */
	public void generateAuto(){
		for (int i = 0; i < list_.size(); i ++){
			Node temp = list_.get(i);
			if (temp.type == 's'){
				if (temp.timeout != 0){
					super.addSequential(temp.command, temp.timeout);
				} else {
					super.addSequential(temp.command);
				}
			} else if (temp.type == 'p'){
				if (temp.timeout != 0){
					super.addParallel(temp.command, temp.timeout);
				} else {
					super.addParallel(temp.command);
				}
			}else {
				System.out.println("Error : type is not s/p");
			}
		}
	}
	

	/**
	 * returns a list of the drive paths in this auto
	 * @return
	 * list of the drive paths in this auto
	 */
	public ArrayList<MP_DrivePath> getPathArray(){
		ArrayList<MP_DrivePath> ret = new ArrayList<MP_DrivePath>();
		for (int i = 0; i < list_.size(); i++){
			Command temp = list_.get(i).command;
			if (temp instanceof MP_PathFollowerCommand){
				ret.add(((MP_PathFollowerCommand) temp).getPath());
			}
		}
		return ret;
	}
	
}
