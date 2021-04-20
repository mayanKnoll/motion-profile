package Improved_MP;

public abstract class MP_Transition extends MP_Tank_DrivePath{

	protected double VRadius_, VStraight_, exelAcc_, stopAcc_, totalTime_, radius_; 
	protected boolean Vsign_, rightOrLeft_;
		
	public double getX(double time){
		return NumericIntegral.numericIntegral(
				new NumericIntegral.Function() {
					double Rsign;
					
					public double function(double t) {
						return getV(t) * Math.sin(getAngleRadians(t));
					}
				},
				0, time,
				0.00001);
	}
	
	public double getTotalX() {
		return getX(getTotalTime());
	}

	public double getY(double time){
		return NumericIntegral.numericIntegral(
				new NumericIntegral.Function() {
					
					public double function(double t) {
						return getV(t) * Math.cos(getAngleRadians(t));
					}
				},
				0, time,
				0.00001);
	}
	
	public double getTotalY() {
		return getY(getTotalTime());
	}
	
	public double getTotalAngleDegrees() {
		return Math.toDegrees(getAngleRadians(totalTime_));
	}

	public double getTotalAngleDegreesWithSignIndicatingLeftOrRight() {
		double rSign = radius_ > 0? 1 : -1;
		return Math.abs(Math.toDegrees(getAngleRadians(totalTime_))) * rSign;
	}
	
	@Override
	public double getAngleRadians(double time){
		//double Vsign = getV(time) > 0? 1 : -1;
		return (left_.getCurrentState(time).pos - right_.getCurrentState(time).pos)
				/ MP_Constants.ROBOT_WIDTH;
		
		
		//double dv = Math.abs(V_ - left_.getV0());
		//return Vsign * (((2 * dv * time) - Math.abs(acc_) * time * time) / MP_Constants.ROBOT_WIDTH);
		//return Vsign * (Math.abs(acc_) * time * time) / MP_Constants.ROBOT_WIDTH;
	}
	
	@Override
	public double getAngleSimulator(double time) {
		//double angleSign = Vsign_ ^ radius_ > 0? -1 : 1;
		//return angleSign * Math.abs(getAngleRadians(time));
		return getAngleRadians(time);
	}

	@Override
	public double getVmax() {
		return Math.abs(VStraight_) > Math.abs(VRadius_)? VStraight_ : VRadius_;
	}

	@Override
	public double getExelerationAcc() {
		return exelAcc_;
	}
	
	@Override
	public double getStopingAcc() {
		return stopAcc_;
	}
	
	public double getV(double time) {
		return (right_.getCurrentState(time).vel + left_.getCurrentState(time).vel) / 2;
	}
}
