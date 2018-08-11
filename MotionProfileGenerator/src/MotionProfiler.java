import java.util.ArrayList;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MotionProfiler {
	// Mechanism characteristics
	private double maxVel;
	private double maxAccel;
	private double maxJerk;
	
	// Calculated constants
	private double timeToMaxAchievableVel;
	private double timeToMaxAchievableVelCoast;
	
	private double timeToMaxAchievableAccel;
	private double timeToMaxAchievableAccelCoast;
	private double timeToAccelCoastDeccel;
	
	private double displacementAccelCoastDeccel;
	private double timeToAccelCoastDeccelCoast;
	
	public MotionProfiler(double maxVel, double maxAccel, double maxJerk) {
		this.maxVel = Math.abs(maxVel);
		this.maxAccel = Math.abs(maxAccel);
		this.maxJerk = Math.abs(maxJerk);
	}
	
	public ArrayList<double[]> generateMotionProfile(double distance, double dt) {
		calculateConstants(distance);
		double timeToComplete = timeToAccelCoastDeccelCoast + timeToAccelCoastDeccel;
		ArrayList<double[]> points = new ArrayList<double[]>();

		// vel at time t is found from algebraic integration, while position is found from numerical integration
		double v1 = maxJerk * timeToMaxAchievableAccel * timeToMaxAchievableAccel / 2; // vel at maxAccel
		double v2 = v1 + maxJerk * timeToMaxAchievableAccel * timeToMaxAchievableAccelCoast; // vel after accel coast
		double v3 = v2 + v1; // vel after accel-coast-deccel
		// skipping a portion since velocity is constant during this period
		double v4 = v3 - v1; // vel after accel-coast-deccel-zero-accel
		double v5 = v3 - v2; // vel after accel-coast-deccel-zero-accel-coast
		
		double currTime = 0;
		double currPos = 0;
		
		// {time, vel, pos}
		double[] currPoint = new double[3];
		
		while (currTime <= timeToComplete) {
			currPoint[0] = currTime;
			
			if (currTime > timeToComplete - timeToMaxAchievableAccel) {
				double t6 = timeToComplete - timeToMaxAchievableAccel;
				currPoint[1] = v5 + lookUpAccel(currTime - t6) * (currTime - t6) / 2 + lookUpAccel(timeToComplete - timeToMaxAchievableAccel) * (currTime - t6);
			} else if (currTime > timeToAccelCoastDeccelCoast + timeToMaxAchievableAccel) {
				currPoint[1] = v4 + (lookUpAccel(currTime) * (currTime - (timeToAccelCoastDeccelCoast + timeToMaxAchievableAccel)));
			} else if (currTime > timeToAccelCoastDeccelCoast) {
				currPoint[1] = v3 + (lookUpAccel(currTime) * (currTime - timeToAccelCoastDeccelCoast)) / 2;
			} else if (currTime > timeToAccelCoastDeccel) {
				currPoint[1] = v3;
			} else if (currTime > timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast) {
				double t2 = timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast;
				currPoint[1] = v2 + (currTime - t2) * (lookUpAccel(currTime) + lookUpAccel(t2)) / 2;
			} else if (currTime > timeToMaxAchievableAccel) {
				currPoint[1] = v1 + lookUpAccel(currTime) * (currTime - timeToMaxAchievableAccel);
			} else if (currTime >= 0) {
				currPoint[1] = lookUpAccel(currTime) * currTime / 2;
			}
			
			currPoint[2] = currPos += currPoint[1] * dt;
			
			double[] currPoints2 = {currPoint[0], currPoint[1], currPoint[2]};
			points.add(currPoints2);
			
			currTime += dt;
		}
		return points;
	}
	
	// default 10ms dt
	public ArrayList<double[]> generateMotionProfile(double distance) {
		return generateMotionProfile(distance, 0.010);
	}
	
	private void calculateConstants(double distance) {
		timeToMaxAchievableVel = Math.min(Math.sqrt(distance / maxAccel), maxVel / maxAccel);
		double maxAchieveableVel = timeToMaxAchievableVel * maxAccel;
		timeToMaxAchievableVelCoast = distance / (maxAccel * timeToMaxAchievableVel) - timeToMaxAchievableVel;
		
		timeToMaxAchievableAccel = Math.min(Math.min(Math.sqrt(maxAchieveableVel / maxJerk), maxAccel / maxJerk),
				(timeToMaxAchievableVel + timeToMaxAchievableVelCoast) / 2);
		timeToMaxAchievableAccelCoast = Math.min(timeToMaxAchievableVel + timeToMaxAchievableVelCoast - 2 * timeToMaxAchievableAccel,
				maxAchieveableVel / (maxJerk * timeToMaxAchievableAccel) - timeToMaxAchievableAccel);
		timeToAccelCoastDeccel = 2 * timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast;
		
		displacementAccelCoastDeccel = timeToAccelCoastDeccel * maxJerk * timeToMaxAchievableAccel * (timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast) / 2;
		timeToAccelCoastDeccelCoast = timeToAccelCoastDeccel +
				(distance - 2 * displacementAccelCoastDeccel) / (maxJerk * timeToMaxAchievableAccel * (timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast));
	}
	
	private double lookUpAccel(double time) {
		if (time > timeToAccelCoastDeccelCoast + timeToAccelCoastDeccel || time <= 0) return 0;
		if (time > timeToAccelCoastDeccelCoast + timeToAccelCoastDeccel - timeToMaxAchievableAccel) return -maxJerk * (timeToAccelCoastDeccelCoast + timeToAccelCoastDeccel - time);
		if (time > timeToAccelCoastDeccelCoast + timeToMaxAchievableAccel) return -maxJerk * timeToMaxAchievableAccel;
		if (time > timeToAccelCoastDeccelCoast)	return -maxJerk * (time - timeToAccelCoastDeccelCoast);
		if (time > timeToAccelCoastDeccel) return 0;
		if (time > timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast) return maxJerk * (timeToAccelCoastDeccel - time);
		if (time > timeToMaxAchievableAccel) return maxJerk * timeToMaxAchievableAccel;
		if (time > 0) return maxJerk * time;
		else return 0;
	}
	
	// Getters and Setters
	public double getMaxVel() {
		return maxVel;
	}
	
	public void setMaxVel(double maxVel) {
		this.maxVel = maxVel;
	}
	
	public double getMaxAccel() {
		return maxAccel;
	}
	
	public void setMaxAccel(double maxAccel) {
		this.maxAccel = maxAccel;
	}
	
	public double getMaxJerk() {
		return maxJerk;
	}
	
	public void setMaxJerk(double maxJerk) {
		this.maxJerk = maxJerk;
	}
	
	public static void main(String[] args) {
		MotionProfiler test = new MotionProfiler(0.54, 0.5, 1);
		ArrayList<double[]> points = test.generateMotionProfile(0.84);

		String path = "C:/Users/Chandra/Documents/points.csv";
		FileWriter fw = null;
		
		try {
			File f = new File(path);
			
			if (f.exists()) {
				f.delete();
			}
			
			fw = new FileWriter(f, true);
			fw.write("");
			fw.write("time, velocity, position\n");
			fw.flush();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		for (int i = 0; i < points.size(); i++) {
        	String result = Double.toString(points.get(i)[0]) + "," + Double.toString(points.get(i)[1]) + "," + Double.toString(points.get(i)[2]) + "\n";
        	
        	try {
        		fw.write(result);
        	} catch (IOException e) {
        		e.printStackTrace();
        	}
        }
        
        try {
			fw.flush();
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
