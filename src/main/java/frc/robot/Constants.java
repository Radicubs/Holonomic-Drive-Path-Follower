package frc.robot;

public class Constants {
    // Speeds in m/s
    // Angular Speeds in rad/s

    public class Simulation{
        // Set these to higher than the corresponding PATH fields in TrajectoryFollower
        // It gives the simulation some buffer if the robot falls behind
        public final static double MAX_ACCELERATION = 5.0;
        public final static double MAX_ANGULAR_ACCELERATION = 4.0;
        public final static double MAX_AXIS_SPEED = 6;
        public final static double MAX_ANGULAR_SPEED = 6.5;
    }

    public class TrajectoryFollower {
        public final static double MAX_PATH_SPEED = 3.0;
        public final static double MAX_PATH_ANGULAR_SPEED = 3.0;
        public final static double MAX_PATH_ANGULAR_ACCELERATION = 2.0;
        public final static double MAX_PATH_ACCELERATION = 2.0;

        // PID Constants
        public final static double xControllerKP = 2.0;
        public final static double xControllerKI = 0.0;
        public final static double xControllerKD = 0.0;

        public final static double yControllerKP = 2.0;
        public final static double yControllerKI = 0.0;
        public final static double yControllerKD = 0.0;

        public final static double thetaControllerKP = 2.0;
        public final static double thetaControllerKI = 0.0;
        public final static double thetaControllerKD = 0.0;

        // Tolerances to end command
        public final static double xToleranceMeters = 0.03;
        public final static double yToleranceMeters = 0.03;
        public final static double rotToleranceDeg = 2;
    }
}
