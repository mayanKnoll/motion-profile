package Improved_MP;

import java.text.DecimalFormat;

import Improved_MP.MP_Path;

public class MP_S2R extends MP_Transition{

	
	/*
	 * For a right turn put + in radius. For a left turn put - in radius.
	 */
	public MP_S2R(double radius, double V, double acc, boolean rightLeft) {
		set(radius, V, V, acc, acc, rightLeft);
	}
	
	public MP_S2R(double radius, double VS, double VR, double exelAcc, double stopAcc, boolean rightLeft) {
		set(radius, VS, VR, exelAcc, stopAcc, rightLeft);
	}
	
	private void set (double radius, double VS, double VR, double exelAcc, double stopAcc,
			boolean rightLeft) {
		double leftRatio = 1, rightRatio = 1;
		
		Vsign_ = VS > 0;
		exelAcc_ = exelAcc;
		stopAcc_ = stopAcc;
		VRadius_ = VR;
		double vrL = VR, vrR = VR;
		VStraight_ = VS;
		radius_ = radius;
		rightOrLeft_ = rightLeft;
		
		if (radius > 0){
			// right turn
			leftRatio = MP_Radius.bigRatio(radius); 
			rightRatio = MP_Radius.smallRatio(radius);
		}else if (radius < 0){
			// left turn
			radius = Math.abs(radius);
			leftRatio = MP_Radius.smallRatio(radius);
			rightRatio = MP_Radius.bigRatio(radius);
		} else if (radius == 0) {
			vrL = Math.abs(vrL);
			vrR = Math.abs(vrR);
			if (rightLeft) {
				vrR *= -1;
			} else {
				vrL *= -1;
			}
		}
	
		left_ = new MP_Path(VS, vrL * leftRatio, exelAcc, stopAcc);
		right_ = new MP_Path(VS, vrR * rightRatio, exelAcc, stopAcc);
		
		if (left_.getTotalTime() > right_.getTotalTime()) {
			
			if (right_.getTotalTime() == 0) {
				right_ = new MP_Path(VS * left_.getTotalTime(), VS, VS, VS, 1, 1);
			} else {

				right_ = new MP_Path(VS, vrR * rightRatio, (VR * rightRatio - VS) / left_.getTotalTime());
			}
			
		} else if (left_.getTotalTime() < right_.getTotalTime()){
			
			if (left_.getTotalTime() == 0) {
				left_ = new MP_Path(VS * right_.getTotalTime(), VS, VS, VS, 1, 1);

			} else {
				left_ = new MP_Path(VS, vrL * leftRatio, (VR * leftRatio - VS) / right_.getTotalTime());
			}
			
		}
		
		totalTime_ = left_.getTotalTime();	
	}
	
	
	@Override
	public String toString() {
		DecimalFormat f = new DecimalFormat("##.##");
		String s = "S2R     : " + f.format(radius_) + ", " + f.format(VStraight_) + ", " + f.format(VRadius_) + ", "
				+ f.format(exelAcc_) + ", " + f.format(stopAcc_);
		return s;
	}


	@Override
	public double getAbsulutVend() {
		// TODO Auto-generated method stub
		return VRadius_;
	}


	@Override
	public double getV0() {
		// TODO Auto-generated method stub
		return VStraight_;
	}
	
}
