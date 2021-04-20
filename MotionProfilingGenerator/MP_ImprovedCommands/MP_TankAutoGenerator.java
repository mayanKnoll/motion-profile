package MP_ImprovedCommands;

import Improved_MP.MP_Constants;
import Improved_MP.MP_R2S;
import Improved_MP.MP_Radius;
import Improved_MP.MP_S2R;
import Improved_MP.MP_Straight;
import Improved_MP.MP_Transition;
import MP_PathFollowers.MP_TankPathFollower;
import MotionProfiling.MPDriveController;
import MotionProfiling.MPGains;
import edu.wpi.first.wpilibj.command.Command;
import Improved_MP.MP_Tank_DrivePath;

public class MP_TankAutoGenerator extends MP_SuperAutoGenerator{
	
	
	public void setDefaultDriveControllerAndGains(MPDriveController dc, MPGains gains){
		dc_ = dc;
		gains_ = gains;
	}

	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * V0  		-			V0 of the move.
	 * 						except of :
	 * 							if Vend of last move is 0 	: 	V0 will be 0 anyway. 
	 * 							if last move is straight	: 	V0 will be last move Vend.
	 * 							if minus 				 	: 	V0 will be last move Vend.
	 * @param
	 * Vmax 	- 			maximum speed.
	 * @param
	 * Vend 	- 			endling speed.
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addStraight(double distance, double V0, double Vmax, double Vend, double exelAcc, double stopAcc,
			MPDriveController dc, MPGains gains)
			throws Exception{
		int lastDrivePathIndex = getLastDriveCommand();
		MP_Tank_DrivePath preMove;
		MP_Straight newMove;
		
		Vmax 	*= MP_Constants.MAX_SPEED;
		Vend 	*= MP_Constants.MAX_SPEED;
		V0 		*= MP_Constants.MAX_SPEED;
		
		exelAcc *= MP_Constants.MAX_EXELRATION_ACC;
		stopAcc *= MP_Constants.MAX_STOPPING_ACC;
		
		if (lastDrivePathIndex == -1){
			newMove = new MP_Straight(distance, Vmax, 0, Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "your move is not legal 2";
				
				throw new Exception(errMes);
			}
			
			addSeqCommand(new MP_TankPathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		preMove = (MP_Tank_DrivePath) ((MP_TankPathFollower) list_.get(lastDrivePathIndex).command).getPath();
				
		if (preMove.getAbsulutVend() == 0){
			newMove = new MP_Straight(distance, Vmax, 0, Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "your move is not legal 3";
				
				throw new Exception(errMes);
			}
			
			addSeqCommand(new MP_TankPathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		if (preMove instanceof MP_Straight){
			newMove = new MP_Straight(
					distance, Vmax, preMove.getAbsulutVend(), Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "Considuring the previus Vend your move is not legal."
						+ "Try to decrease the deltaV between the two moves.";
				
				throw new Exception(errMes);
			}
			
			addSeqCommand(new MP_TankPathFollower(
					dc, 
					newMove, 
					gains));
			
		} else if (preMove instanceof MP_Radius){
			if (V0 < 0) {
				/*
				 * Because this is straight after radius if we will put the same relation
				 * between the maximum speeds the speed will increase during the transition
				 * and because exelAcc is less then stopAcc the transition will take more time.
				 * That is why here we put the exact same speed. 
				 */
				V0 = preMove.getAbsulutVend();
			}
			
			MP_R2S trans = new MP_R2S(((MP_Radius) preMove).getRadius(),
					preMove.getAbsulutVend(), V0,
					exelAcc, stopAcc, preMove.getTotalAngleDegrees() > 0);
			
			System.out.println(preMove.getTotalAngleDegrees());
			
			double preMoveAngle = ((MP_Radius) preMove).getTotalAngleDegrees();
			double transAngle = Math.abs(trans.getTotalAngleDegrees()) * (preMoveAngle > 0 ? 1 : -1);
						
			if (Math.abs(transAngle) > Math.abs(preMoveAngle)){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The transition angle is more then the radius before\n"
						+ "Please try one of those options : \n"
						+ "\t- increase radius\n"
						+ "\t- decrease radius Vend\n";
				
				throw new Exception(errMes);
			}
			
			MP_Radius newRadius = new MP_Radius(
					((MP_Radius) preMove).getRadius(),
					preMoveAngle - transAngle,
					preMove.getVmax(),
					preMove.getV0(),
					preMove.getAbsulutVend(),
					preMove.getExelerationAcc(),
					preMove.getStopingAcc());
			
			if (!newRadius.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The previus Radius modification is not legal"
						+ "Please try one of those options : \n"
						+ "\t- increase radius\n"
						+ "\t- decrease radius Vend\n";
				
				throw new Exception(errMes);
			}
			
			// preperring the new move
			newMove = new MP_Straight(
					distance, Vmax, V0, Vend, exelAcc, stopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addStraight(" + distance
						+ ", " + Vmax + ", " + Vend + ", " + exelAcc + "\n"
						+ "The new move is not legal 4";
				
				throw new Exception(errMes);
			}
			
			// changing the radius before to match the transition
			list_.set(lastDrivePathIndex
					, new Node(new MP_TankPathFollower(dc, newRadius, gains), 's'));
			
			//adding the transition afters
			addSeqCommand(new MP_TankPathFollower(dc, trans, gains));
			
			//adding the new Move
			addSeqCommand(new MP_TankPathFollower(dc, newMove, gains));
		}
	}
	
	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * V0  		-			V0 of the move.
	 * 						except of :
	 * 							if Vend of last move is 0 	: 	V0 will be 0 anyway. 
	 * 							if last move is straight	: 	V0 will be last move Vend.
	 * 							if minus 				 	: 	V0 will be last move Vend.
	 * @param
	 * Vmax 	- 			maximum speed.
	 * @param
	 * Vend 	- 			endling speed.
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 */
	public void addStraight(double distance, double V0, double Vmax, double Vend, double exelAcc, double stopAcc)
			throws Exception{
		addStraight(distance, V0, Vmax, Vend, exelAcc, stopAcc, dc_, gains_);
	}
	
	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * Vend 	- 			endling speed.
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addStraight(double distance, double Vend, MPDriveController dc,
			MPGains gains) throws Exception{
		addStraight(distance, -1, 1, Vend, 1, 1, dc, gains); // -1 in V0 means last Vend
	}
	
	/**
	 * @param 
	 * distance -	 		put +/- for forward/backword in order.
	 * @param
	 * Vend 	- 			endling speed.
	 */
	public void addStraight(double distance, double Vend) throws Exception{
		addStraight(distance, Vend, dc_, gains_);
	}
	
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * V0		-		V0 of the move
	 * 					except for :
	 * 						if V0 of last move is 0		:	V0 will be 0.
	 * 						if minus 					: 	V0 will be last move Vend.	
	 * @param
	 * Vmax 	- 		Vmax of the MIDDLE of the robot!
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addRadius(double radius, double angle, double V0, double Vmax, double Vend,
			double exelAcc, double stopAcc, MPDriveController dc, MPGains gains)
					throws Exception{
		int lastDrivePathIndex = getLastDriveCommand();
		MP_Tank_DrivePath preMove;
		MP_Tank_DrivePath newMove;
		
		Vmax 	*= MP_Radius.getMaxSpeedForRadius(radius);
		Vend	*= MP_Radius.getMaxSpeedForRadius(radius);
		V0		*= MP_Radius.getMaxSpeedForRadius(radius);
		
		double radiusExelAcc = exelAcc * MP_Radius.getMaxAcc(MP_Constants.MAX_EXELRATION_ACC, radius);
		double radiusStopAcc = stopAcc * MP_Radius.getMaxAcc(MP_Constants.MAX_STOPPING_ACC, radius);
		
		
		if (lastDrivePathIndex == -1){
			newMove = new MP_Radius(radius, angle, Vmax, 0, Vend, radiusExelAcc, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The move you tryed to do is not legal";
				throw new Exception(errMes);
			}
			
			addSeqCommand(new MP_TankPathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		preMove = (MP_Tank_DrivePath) ((MP_TankPathFollower) list_.get(lastDrivePathIndex).command).getPath();
				
		if (preMove.getAbsulutVend() == 0){
			newMove = new MP_Radius(radius, angle, Vmax, 0, Vend, radiusExelAcc, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The move you tryed to do is not legal";
				throw new Exception(errMes);
			}
			
			addSeqCommand(new MP_TankPathFollower(dc,
					newMove,
					gains));
			return;
		}
		
		if (preMove instanceof MP_Straight){
			if (V0 < 0) {
				// the Vend related to the maximum speed (from 0 to 1)
				V0 = preMove.getAbsulutVend() / MP_Constants.MAX_SPEED;
				
				// the related speed for the radius
				V0 *= MP_Radius.getMaxSpeedForRadius(radius);
			}		
			
			MP_S2R trans = new MP_S2R(	radius,
										preMove.getAbsulutVend(),
										V0,
										exelAcc * MP_Constants.MAX_EXELRATION_ACC,
										stopAcc * MP_Constants.MAX_STOPPING_ACC,
										angle > 0);
			
			
			double transAngle = Math.abs(trans.getTotalAngleDegrees()) * (angle > 0 ? 1 : -1);

			if (Math.abs(angle) < Math.abs(transAngle)){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The transition angle is more then the radius angle\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (radius Vend)\n"
						+ "\t- increase radius\n";
				
				throw new Exception(errMes);
			}
	
			newMove = new MP_Radius(
					radius
					, angle - transAngle
					, Vmax
					, V0
					, Vend
					, radiusExelAcc
					, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The new radius (considuring the transmition angle) is not legal\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (radius Vend)\n"
						+ "\t- increase radius\n";
				
				throw new Exception(errMes);
			}
			
			addSeqCommand(new MP_TankPathFollower(dc, trans, gains));
			addSeqCommand(new MP_TankPathFollower(dc, newMove, gains));
			
		} else if (preMove instanceof MP_Radius){
			if (!(radius > 0 ^ ((MP_Radius) preMove).getRadius() > 0)){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "You cant combine two radiuses in the same direction";
				
				throw new Exception(errMes);	
			}
			
			if (V0 < 0) {
				V0 = preMove.getAbsulutVend();
			}
			
			MP_R2S first = new MP_R2S(	((MP_Radius) preMove).getRadius(),
										preMove.getAbsulutVend(),
										(preMove.getAbsulutVend() + V0) / 2,
										exelAcc * MP_Constants.MAX_EXELRATION_ACC,
										stopAcc * MP_Constants.MAX_STOPPING_ACC,
										preMove.getTotalAngleDegrees() > 0);
			
			MP_S2R second = new MP_S2R(	radius, 
										(preMove.getAbsulutVend() + V0) / 2, 
										V0,
										exelAcc * MP_Constants.MAX_EXELRATION_ACC,
										stopAcc * MP_Constants.MAX_STOPPING_ACC,
										angle > 0);
			
			double firstAngle = Math.abs(first.getTotalAngleDegrees()) * (angle > 0 ? 1 : -1);
			double secondAngle = Math.abs(second.getTotalAngleDegrees()) * (angle > 0? 1 : -1);
			double lastMoveAngle = Math.abs(preMove.getTotalAngleDegrees());

			System.out.println("1");
			
			
			if (lastMoveAngle < firstAngle){
				if (angle < firstAngle + secondAngle){
					String errMes = "Cant make this move : addRadius(" + radius
							+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
							+ "the angle of the previus move is less then the transmition angle.\n"
							+ "Please try one of those options : \n"
							+ "\t- decrease transition speed (radius Vend)\n"
							+ "\t- increase radius\n";
					
					throw new Exception(errMes);
				}
				
			} else if (angle < secondAngle){
				if (lastMoveAngle < firstAngle + secondAngle){
					String errMes = "Cant make this move : addRadius(" + radius
							+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
							+ "the angle of the mew move is less then the transmition angle.\n"
							+ "Please try one of those options : \n"
							+ "\t- decrease transition speed (radius Vend)\n"
							+ "\t- increase radius\n";
					
					throw new Exception(errMes);
				}
			}
			MP_Radius updatePreMove = new MP_Radius(((MP_Radius) preMove).getRadius()
					, preMove.getTotalAngleDegrees() - firstAngle
					, preMove.getVmax()
					, preMove.getV0()
					, preMove.getAbsulutVend()
					, preMove.getExelerationAcc()
					, preMove.getStopingAcc());
			
			if (!updatePreMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The modification of the previus radius (considuring the transmition angle) is not legal\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (radius Vend)\n"
						+ "\t- increase radius\n";
				
				throw new Exception(errMes);
			}
			
			newMove = new MP_Radius(radius
					, angle - secondAngle
					, Vmax
					, V0
					, Vend
					, radiusExelAcc
					, radiusStopAcc);
			
			if (!newMove.isLegal()){
				String errMes = "Cant make this move : addRadius(" + radius
						+ ", " + angle + ", " + Vmax + ", " + Vend + ", " + exelAcc + ");\n"
						+ "The new radius (considuring the transmition angle) is not legal\n"
						+ "Please try one of those options : \n"
						+ "\t- decrease transition speed (previus radius Vend)\n"
						+ "\t- increase radius (new radius)\n";
				
				throw new Exception(errMes);
			}
			
			// changing the radius before to match the transition
			list_.set(lastDrivePathIndex
					, new Node(new MP_TankPathFollower(dc, updatePreMove, gains), 's')); // need to be fixed
			
			// adding the transmitions
			addSeqCommand(new MP_TankPathFollower(dc, first, gains));
			addSeqCommand(new MP_TankPathFollower(dc, second, gains));
			
			//adding the new move
			addSeqCommand(new MP_TankPathFollower(dc, newMove, gains));
		}
	}
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * V0		-		V0 of the move
	 * 					except for :
	 * 						if V0 of last move is 0		:	V0 will be 0.
	 * 						if minus 					: 	V0 will be last move Vend.	
	 * @param
	 * Vmax 	- 		Vmax of the MIDDLE of the robot!
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 * @param
	 * exelAcc 	- 		speeding acceleration of the MIDDLE of the robot!
	 * @param
	 * stopAcc	-		stopping acceleration of the MIDDLE of the robot!
	 */
	public void addRadius(double radius, double angle, double V0, double Vmax, double Vend,
			double exelAcc, double stopAcc) throws Exception{
		addRadius(radius, angle, V0, Vmax, Vend, exelAcc, stopAcc, dc_, gains_);
	}
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 * @param
	 * dc		-		the drive controller for this move
	 * @param
	 * gains	-		the pid and kv, ka for this move
	 */
	public void addRadius(double radius, double angle, double Vend,
			MPDriveController dc, MPGains gains) throws Exception{
		
		addRadius(radius, angle, -1, 1, Vend, 1, 1, dc, gains); // -1 in V0 means last move Vend
	}
	
	/**
	 *@param 
	 * radius	-		put +/- to choose forward/backword in order.
	 * @param
	 * angle 	- 		put +/- to choose left/right in order.
	 * @param
	 * Vend 	- 		Vend of the MIDDLE of the robot!
	 */
	public void addRadius(double radius, double angle, double Vend) throws Exception{
		addRadius(radius, angle, Vend, dc_, gains_);
	}
	
	public void printVel(boolean leftOrRigth) {
		if (leftOrRigth) {
			System.out.println("Vel Left");
		} else {
			System.out.println("Vel Right");
		}
		
		for (int i = 0; i < list_.size(); i++){
			Command temp = list_.get(i).command;
			if (temp instanceof MP_TankPathFollower){
				System.out.println(((MP_TankPathFollower)temp).getPath().toString());

				if (leftOrRigth) {
					((MP_Tank_DrivePath) ((MP_TankPathFollower)temp).getPath()).getLeftPath().printPath();
				} else {
					((MP_Tank_DrivePath) ((MP_TankPathFollower)temp).getPath()).getRightPath().printPath();
				}
			}
		}
	}
	
	/**
	 *  call the toString method for each driving command
	 */
	public void printMoves(){
		System.out.println("Print Moves");
		for (int i = 0; i < list_.size(); i++){
			Command temp = list_.get(i).command;
			if (temp instanceof MP_TankPathFollower){
				System.out.println(((MP_TankPathFollower)temp).getPath().toString());
			}
		}
	}
	
	/**
	 * needs to print the moves in simulator format
	 */
	public void printMovesSimulator(){
		double totalTime = 0;
		//System.out.println("Moves for Simulator : ");
		System.out.println("1.06");
		for (int i = 0; i < list_.size(); i++){
			Command temp = list_.get(i).command;
			if (temp instanceof MP_TankPathFollower){
				totalTime += ((MP_TankPathFollower)temp).getPath().getTotalTime();
				MP_Tank_DrivePath temp2 = (MP_Tank_DrivePath) ((MP_TankPathFollower) temp).getPath();
				if (temp2 instanceof MP_Transition){
					System.out.printf("t,%.20f,%.20f,%.20f", ((MP_Transition)temp2).getTotalX()
							,((MP_Transition)temp2).getTotalY()
							,((MP_Transition)temp2).getTotalAngleDegreesWithSignIndicatingLeftOrRight());
					System.out.println();
				} else if (temp2 instanceof MP_Radius){
					System.out.printf("r,%.20f,%.20f",((MP_Radius)temp2).getRadius(),((MP_Radius)temp2).getTotalAngleDegrees());
					System.out.println();
				} else if (temp2 instanceof MP_Straight){
					System.out.printf("s,%.20f", ((MP_Straight)temp2).getDistance());
					System.out.println();
				} 
			}
		}
		System.out.println(totalTime);
	}
	
	/**
	 *  prints and returns the last drive command total time
	 * @return
	 * last move total time
	 */
	public double lastMoveTotalTime(){
		System.out.println("Last Move Total Time :");
		int lastMoveIndex = getLastDriveCommand();
		if (lastMoveIndex == -1){
			System.out.println("there is no last move!");
			return 0;
		}
		
		double totalTime = ((MP_TankPathFollower)list_.get(lastMoveIndex).command)
				.getPath().getTotalTime();
		System.out.println(totalTime);
		return totalTime;
	}
	
}
