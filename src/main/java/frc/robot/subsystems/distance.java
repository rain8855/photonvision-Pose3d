package frc.robot.subsystems;

import java.security.PublicKey;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.math.geometry.proto.Pose3dProto;


public class distance extends SubsystemBase{    
    private PhotonCamera camera = new PhotonCamera("limelight");
    Pose2d pose2d = new Pose2d(15.445, 1.448, new Rotation2d(-180));
    public static  double kCameraHeight = 14.5;
    public static  double kTargetPitch = 0;
    public static  double kTargetHeight = 0;
    public static  double kCameraPitch = 22.5;
    public static  double CAMERA_HEIGHT_METERS = 0.145;
    public static  double TARGET_HEIGHT_METERS = 0;
    public static  double CAMERA_PITCH_RADIANS = 22.5;
    public static  double forwardSpeed = 0 ;

    private final XboxController xboxController = new XboxController(0);
    private PhotonPipelineResult result = camera.getLatestResult();
    public boolean hasTargets = result.hasTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    public static double GOAL_RANGE_METERS = calculateDistanceToTargetMeters();
    
    
    
    public distance(){
        // Calculate robot's field relative pose    
        Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
            kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw())
                                                        , gyro.getRotation2d(), targetPose, cameraToRobot);
        // Calculate robot's field relative pose
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget()
                        , aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
        if (xboxController.getAButton()) {
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));
                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                forwardSpeed = -Controller.calculate(range,GOAL_RANGE_METERS);
            }
        }
    }

    private static double calculateDistanceToTargetMeters() {
        double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'calculateDistanceToTargetMeters'");
    }
}


