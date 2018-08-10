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

		System.out.println("time to complete: " + timeToComplete);

		double v1 = maxJerk * timeToMaxAchievableAccel * timeToMaxAchievableAccel / 2; // vel at maxAccel
		double v2 = v1 + maxJerk * timeToMaxAchievableAccel * timeToMaxAchievableAccelCoast; // vel after accel coast
		double v3 = v2 + v1; // vel after accel-coast-deccel
		// skipping a portion since velocity is constant during this period
		double v4 = v3 - v1; // vel after accel-coast-deccel-zero-accel
		double v5 = v3 - v2; // vel after accel-coast-deccel-zero-accel-coast
		
		double p1 = maxJerk * timeToMaxAchievableAccel * timeToMaxAchievableAccel * timeToMaxAchievableAccel / 6; // pos at maxAccel
		double p2 = p1 + (v2 + v1) * timeToMaxAchievableAccelCoast / 2;
		double p3 = displacementAccelCoastDeccel;
		double p4 = p3 + (v3 * timeToAccelCoastDeccelCoast - timeToAccelCoastDeccel);
		double p5 = p4 + (p3 - p2);
		double p6 = p5 + (p2 - p1);
				
		double currTime = 0;
		double currVel = 0;
		double currPos = 0;
		
		double[] currPoint = new double[3];
		
		while (currTime <= timeToComplete) {
			currPoint[0] = currTime;
			
			if (currTime > timeToComplete - timeToMaxAchievableAccel) {
				double t6 = timeToComplete - timeToMaxAchievableAccel;
				currVel = v5 + lookUpAccel(currTime - t6) * (currTime - t6) / 2 + lookUpAccel(timeToComplete - timeToMaxAchievableAccel) * (currTime - t6);
//				currPos = p6 + lookUpAccel(currTime - t6) * (currTime - t6) * (currTime - t6) / 6 +
//						lookUpAccel(timeToComplete - timeToMaxAchievableAccel) * (currTime - t6) * (currTime - t6) / 2 +
//						v5 * (currTime - t6);
				currPoint[1] = currVel;
//				currPoint[2] = currPos;
			} else if (currTime > timeToAccelCoastDeccelCoast + timeToMaxAchievableAccel) {
				currVel = v4 + (lookUpAccel(currTime) * (currTime - (timeToAccelCoastDeccelCoast + timeToMaxAchievableAccel)));
//				currPos = p5 + (v4 + currVel) * (currTime - (timeToAccelCoastDeccelCoast + timeToMaxAchievableAccel)) / 2;
				currPoint[1] = currVel;
//				currPoint[2] = currPos;
			} else if (currTime > timeToAccelCoastDeccelCoast) {
				currVel = v3 + (lookUpAccel(currTime) * (currTime - timeToAccelCoastDeccelCoast)) / 2;
//				currPos = p4 + (-lookUpAccel(currTime) * currTime * currTime / 6 + v3 * currTime);
				currPoint[1] = currVel;
//				currPoint[2] = currPos;
			} else if (currTime > timeToAccelCoastDeccel) {
				currVel = v3;
//				currPos = p3 + v3 * (currTime - timeToAccelCoastDeccel);
				currPoint[1] = currVel;
//				currPoint[2] = currPos;
			} else if (currTime > timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast) {
				double t2 = timeToMaxAchievableAccel + timeToMaxAchievableAccelCoast;
				
				currVel = v2 + (currTime - t2) * (lookUpAccel(currTime) + lookUpAccel(t2)) / 2;
//				currPos = p2 + -lookUpAccel(currTime) * (currTime - t2) * (currTime - t2) / 6 +
//						lookUpAccel(t2) * (currTime - t2) * (currTime - t2) / 2 +
//						v2 * (currTime - t2);
				currPoint[1] = currVel;
//				currPoint[2] = currPos;
			} else if (currTime > timeToMaxAchievableAccel) {
				currVel = v1 + lookUpAccel(currTime) * (currTime - timeToMaxAchievableAccel);
//				currPos = p1 + ((currVel + v1) * (currTime - timeToMaxAchievableAccel)) / 2;
				currPoint[1] = currVel;
//				currPoint[2] = currPos;
			} else if (currTime >= 0) {
				currVel = lookUpAccel(currTime) * currTime / 2;
//				currPos = lookUpAccel(currTime) * currTime * currTime / 6;
				currPoint[1] = currVel;
//				currPoint[2] = currPos;
			}
			
			currPos += currVel * dt;
			currPoint[2] = currPos;
			
			double[] currPoints2 = {currPoint[0], currPoint[1], currPoint[2]};
			points.add(currPoints2);
			
			currTime += dt;
		}
		return points;
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
		MotionProfiler test = new MotionProfiler(0.6, 0.5, 1);
		ArrayList<double[]> points = test.generateMotionProfile(1.6, 0.01);

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
		
		int i = 0;
		
		while (i < points.size()) {
        	String result = Double.toString(points.get(i)[0]) + "," + Double.toString(points.get(i)[1]) + "," + Double.toString(points.get(i)[2]) + "\n";
        	
        	try {
        		fw.write(result);
        	} catch (IOException e) {
        		e.printStackTrace();
        	}
        	
        	i++;
        }
        
        try {
			fw.flush();
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
