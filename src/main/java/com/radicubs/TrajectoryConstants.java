package com.radicubs;

public record TrajectoryConstants(
        double maxPathSpeed,
        double maxPathAngularSpeed,
        double maxPathAcceleration,
        double maxPathAngularAcceleration,

        double xControllerkP,
        double xControllerkI,
        double xControllerkD,

        double yControllerkP,
        double yControllerkI,
        double yControllerkD,

        double thetaControllerkP,
        double thetaControllerkI,
        double thetaControllerkD,

        double xTolerance,
        double yTolerance,
        double rotTolerance
) {
    public static final TrajectoryConstants DEFAULTS = new TrajectoryConstants(3.0, 3.0,
            2.0, 2.0, 2.0, 0, 0,
            2.0, 0, 0, 2.0, 0, 0,
            0.03, 0.03, 2);

}
