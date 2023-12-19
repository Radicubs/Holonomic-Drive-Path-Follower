package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HolonomicPathFollower;

public class SimulationChassis extends SubsystemBase implements HolonomicPathFollower {
    private double xVelocity, yVelocity, angVelocity;
    private double targetXVelocity, targetYVelocity, targetAngVelocity;
    private Field2d field2d;
    private Pose2d robotPose;

    public SimulationChassis(){
        field2d = new Field2d();
        SmartDashboard.putData(field2d);
        robotPose = new Pose2d(1, 1, new Rotation2d(0));
        xVelocity = yVelocity = angVelocity = 0;
    }

    @Override
    public void driveFromFieldOrientedChassisSpeeds(ChassisSpeeds fieldRelativeSpeeds){
        targetXVelocity = fieldRelativeSpeeds.vxMetersPerSecond;
        targetYVelocity = fieldRelativeSpeeds.vyMetersPerSecond;
        targetAngVelocity = fieldRelativeSpeeds.omegaRadiansPerSecond;
    }

    public void driveFromRobotOrientedChassisSpeeds(ChassisSpeeds robotOrientedSpeeds){
        //robot oriented to field oriented conversion
        //velocity attributes of class are field oriented
        Translation2d robotOrientedSpeed = new Translation2d(robotOrientedSpeeds.vxMetersPerSecond, robotOrientedSpeeds.vyMetersPerSecond);
        Translation2d fieldOrientedSpeed = robotOrientedSpeed.rotateBy(robotPose.getRotation());

        targetXVelocity = fieldOrientedSpeed.getX();
        targetYVelocity = fieldOrientedSpeed.getY();

        //rotation is rotation, even if you rotate the rotation :)
        targetAngVelocity = robotOrientedSpeeds.omegaRadiansPerSecond;
    }

    public void displayTrajectory(Trajectory trajectory){
        field2d.getObject("trajectory").setTrajectory(trajectory);
    }
    public void displayPoses(Pose2d ...pose2ds){
        field2d.getObject("poses").setPoses(pose2ds);
    }



    public ChassisSpeeds getChassisSpeeds(){
        return new ChassisSpeeds(xVelocity, yVelocity, angVelocity);
    }

    //robot pose heading and display image are 90 deg off
    public Pose2d getRobotPose(){
        return robotPose;
    }

    public void setRobotPose(Pose2d robotPose){
        this.robotPose = robotPose;
    }


    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("X Velocity", xVelocity);
        SmartDashboard.putNumber("Y Velocity", yVelocity);
        SmartDashboard.putNumber("Angular Velocity", angVelocity);

        SmartDashboard.putNumber("Target X Velocity", targetXVelocity);
        SmartDashboard.putNumber("Target Y Velocity", targetYVelocity);
        SmartDashboard.putNumber("Target Angular Velocity", targetAngVelocity);
        double deltaXVel = targetXVelocity - xVelocity;
        double deltaYVel = targetYVelocity - yVelocity;

        double deltaCombinedVel = Math.sqrt(Math.pow(deltaXVel, 2) + Math.pow(deltaYVel, 2));
        // 1s / 20ms = 50
        double targetAcceleration = deltaCombinedVel * 50;

        //Ensure acceleration does not exceed MAX_ACCELERATION for lin & rot movement
        if(targetAcceleration <= Constants.Simulation.MAX_ACCELERATION){
            xVelocity = targetXVelocity;
            yVelocity = targetYVelocity;
        }
        else{
            double scale = Constants.Simulation.MAX_ACCELERATION / targetAcceleration;
            xVelocity += deltaXVel * scale;
            yVelocity += deltaYVel * scale;
        }

        double deltaAngVelocity = targetAngVelocity - angVelocity;
        // 1s / 20ms = 50
        double targetAngAcceleration = Math.abs(deltaAngVelocity) * 50;
        if(targetAngAcceleration <= Constants.Simulation.MAX_ANGULAR_ACCELERATION){
            angVelocity = targetAngVelocity;
        }
        else{
            double scale = Constants.Simulation.MAX_ANGULAR_ACCELERATION / targetAngAcceleration;
            angVelocity += deltaAngVelocity * scale;
        }

        Translation2d velocitySum = robotPose.getTranslation().plus(new Translation2d(xVelocity / 50, yVelocity / 50));
        Rotation2d angularVelocitySum = robotPose.getRotation().plus(new Rotation2d(angVelocity / 50));
        robotPose = new Pose2d(velocitySum, angularVelocitySum);

        field2d.setRobotPose(robotPose);
    }
}