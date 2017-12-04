package org.usfirst.frc.team2659.robot;

import org.usfirst.frc.team2659.robot.util.Rotation2d;
import org.usfirst.frc.team2659.robot.util.Twist2d;

public class RobotStateEstimator {
	 static RobotStateEstimator instance_ = new RobotStateEstimator();

	    public static RobotStateEstimator getInstance() {
	        return instance_;
	    }

	    RobotStateEstimator() {
	    }

	    RobotState robot_state_ = RobotState.getInstance();
	    double left_encoder_prev_distance_ = 0;
	    double right_encoder_prev_distance_ = 0;

	    public synchronized void onStart(double timestamp) {
	        left_encoder_prev_distance_ = RobotMap.leftEncoder.getDistance();
	        right_encoder_prev_distance_ = -RobotMap.rightEncoder.getDistance();
	    }

	    public synchronized void onLoop(double timestamp) {
	        final double left_distance = RobotMap.leftEncoder.getDistance();
	        final double right_distance = -RobotMap.rightEncoder.getDistance();
	        final Rotation2d gyro_angle = RobotMap.getGyroAngle();
	        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
	                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
	        final Twist2d predicted_velocity = Kinematics.forwardKinematics(RobotMap.leftEncoder.getRate(),
	                -RobotMap.rightEncoder.getRate());
	        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
	        left_encoder_prev_distance_ = left_distance;
	        right_encoder_prev_distance_ = right_distance;
	    }

	    public void onStop(double timestamp) {
	        // no-op
	    }
}
