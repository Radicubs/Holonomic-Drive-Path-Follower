package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class Simulation{
        public final static double MAX_ACCELERATION = 5.0; // m/s^2
        public final static double MAX_ANGULAR_ACCELERATION = 4.0; // rad/s^2

        public final static double MAX_PATH_SPEED = 3.0;
        public final static double MAX_PATH_ANGULAR_SPEED = 3.0;
        public final static double MAX_PATH_ANGULAR_ACCELERATION = 2.0;
        public final static double MAX_PATH_ACCELERATION = 2.0;

        public final static double MAX_AXIS_SPEED = 6; // 6 m/s (maximum speed along each axis, max speed is this times sqrt(2))
        public final static double MAX_ANGULAR_SPEED = 2; // rad/s

    }
}
