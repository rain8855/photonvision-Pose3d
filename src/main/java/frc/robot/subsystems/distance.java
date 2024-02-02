package frc.robot.subsystems;

import java.security.PublicKey;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.math.geometry.proto.Pose3dProto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class distance extends SubsystemBase{    
    private PhotonCamera camera = new PhotonCamera("limelight");
    
    private Pose2d Pose2d = new Pose2d(0.0, 0.0, new Rotation2d(-180)); 
    private Pose3d robotPose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
    Transform3d robotToCam = new Transform3d(new Translation3d(0.42, 0, 0.22), new Rotation3d(0,0,0));

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
    Transform3d cameraToRobot = target.getBestCameraToTarget();

    //public static double GOAL_RANGE_METERS = calculateDistanceToTargetMeters();

    private AprilTagFieldLayout aprilTagFieldLayout;

        
    
    
    public double[] estimateFieldToRobotAprilTag(){
        // Calculate robot's field relative pose    
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        // Pose2d robotPose2d = PhotonUtils.estimateFieldToRobot(
        //     kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw())
        //                                             , gyro.getRotation2d(), targetPose, cameraToRobot);
        // Calculate robot's field relative pose

        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);



        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget()
                        , aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
        double robotPoseX = robotPose.toPose2d().getX();
        double robotPoseY = robotPose.toPose2d().getY();
        double [] robotPose_XY = {robotPoseX,robotPoseY};
        return robotPose_XY ;     
        
        
        // if (xboxController.getAButton()) {
        //     // Vision-alignment mode
        //     // Query the latest result from PhotonVision
        //     var result = camera.getLatestResult();

        //     if (result.hasTargets()) {
        //         // First calculate ranges
        //         double range =
        //                 PhotonUtils.calculateDistanceToTargetMeters(
        //                         CAMERA_HEIGHT_METERS,
        //                         TARGET_HEIGHT_METERS,
        //                         CAMERA_PITCH_RADIANS,
        //                         Units.degreesToRadians(result.getBestTarget().getPitch()));
        //         // Use this range as the measurement we give to the PID controller.
        //         // -1.0 required to ensure positive PID controller effort _increases_ range
        //         forwardSpeed = -Controller.calculate(range,GOAL_RANGE_METERS);
        //     }
        // }
    }
    

    // private static double calculateDistanceToTargetMeters() {
    //     double distanceToTarget = PhotonUtils.getDistanceToPose(, targetPose);
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'calculateDistanceToTargetMeters'");
    // }
}


