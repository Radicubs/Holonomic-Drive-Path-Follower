package frc.robot;

public class Constants {
    // Speeds in m/s
    // Angular Speeds in rad/s
    public class Simulation {
        // Set these to higher than the corresponding PATH fields in TrajectoryFollower
        // It gives the simulation some buffer if the robot falls behind
        public final static double MAX_ACCELERATION = 5.0;
        public final static double MAX_ANGULAR_ACCELERATION = 4.0;
    }
}
