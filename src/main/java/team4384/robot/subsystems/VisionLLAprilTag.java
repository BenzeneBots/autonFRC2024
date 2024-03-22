package team4384.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team4384.robot.LimelightHelpers;
import team4384.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionLLAprilTag extends SubsystemBase {

    // Read the Apriltags orientation and position in the the field as defined by WPILIB
    private final AprilTagFieldLayout currentTagFieldLayout;
    private String LimeLightInstance = null;
    private Alliance selectedAlliance = Alliance.Blue;
    private LimelightHelpers.LimelightResults LimeLightInstanceJsonResults;

    private static final String ntInstanceName = "/VisionLL/";
    Pose2d robotPose2DFieldSpaceRelativeToTeamOrigin = new Pose2d();
    Pose3d robotPose3DFieldSpaceRelativeToTeamOrigin = new Pose3d();
    
    double AvgRobotDistanceFromTarget = 0;
    Pose3d botPose = new Pose3d();
    Pose2d desiredPose2d = new Pose2d();

    long[] lastDetectedTagIDs;


    IntegerArrayPublisher publishAprilTagsInRobotView =
            NetworkTableInstance.getDefault().getIntegerArrayTopic(ntInstanceName+"Fiducial").publish();
    DoublePublisher publishAprilTagDistance =
            NetworkTableInstance.getDefault().getDoubleTopic(ntInstanceName+"TagDist").publish();
    StructPublisher<Pose3d> pubFieldPose3D = NetworkTableInstance.getDefault()
            .getStructTopic(ntInstanceName+"FieldPose3d", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> pubFieldPose3dArray = NetworkTableInstance.getDefault()
            .getStructArrayTopic(ntInstanceName+"FieldPose3dArray", Pose3d.struct).publish();
    StructArrayPublisher<Pose2d> pubEstimatedRobotPose2dArray = NetworkTableInstance.getDefault()
            .getStructArrayTopic(ntInstanceName+"EstimatedRobotPose2d", Pose2d.struct).publish();

    public VisionLLAprilTag(Alliance selectedAlliance, String kLimeLight_1) {
        LimeLightInstance = kLimeLight_1;
        
        AprilTagFieldLayout loadFieldLayout;
        try {
            loadFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            loadFieldLayout.setOrigin(selectedAlliance == Alliance.Red ?
                OriginPosition.kRedAllianceWallRightSide : OriginPosition.kBlueAllianceWallRightSide);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load the Field Resource", e.getStackTrace());
            loadFieldLayout = null;
        }
        this.currentTagFieldLayout = loadFieldLayout;
        InitLimeLightSetting();

    }

    private void InitLimeLightSetting() {
        //Set Led Off at Init
        LimelightHelpers.setLEDMode_ForceOff(LimeLightInstance);
        //Set april_tag pipeline for limelight 2
        //ToDo Check AprilTag pipeline number for this Limelight
        LimelightHelpers.setPipelineIndex(LimeLightInstance, 0);
        //Set Camera position relative to Robot Centre. 
        //ToDo - Update arguments as per the final position
        LimelightHelpers.setCameraPose_RobotSpace(LimeLightInstance, 0, 0, 0, 0, 0, 0);
    }

    private boolean validVisionDetected() {
        boolean bCurrentView = LimelightHelpers.getTV(LimeLightInstance);
        //ToDo Check whether vision is within the field. We want to avoid outside views.
        return bCurrentView;
    }

    public Pose3d getRobotPose3D() {
        return LimelightHelpers.getBotPose3d(LimeLightInstance);
    }

    public Pose2d getRobotPose2D() {
        return LimelightHelpers.getBotPose2d(LimeLightInstance);
    }

    public Pose3d getRobotInTargetSpace3D() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(LimeLightInstance);
    }

    public boolean MultipleTagsInView() {
        LimelightTarget_Fiducial[] tags = LimeLightInstanceJsonResults.targetingResults.targets_Fiducials;
        int[] tagIDs = new int[tags.length];
        return (tagIDs.length > 1);
    }


    public double GetAvgRobotDistanceFromTarget() {
        double averageTagDistance = 0.0;
        LimelightTarget_Fiducial[] tags = LimeLightInstanceJsonResults.targetingResults.targets_Fiducials;
        int[] tagIDs = new int[tags.length];
        long[] tempTagIDs = new long[tags.length];
        for (int index = 0; index < tags.length; index++) {
            tagIDs[index] = (int) tags[index].fiducialID;
            tempTagIDs[index] = (int) tags[index].fiducialID;
            averageTagDistance +=
                    tags[index].getTargetPose_CameraSpace().getTranslation().getNorm();
        }
        averageTagDistance /= tagIDs.length;
        this.lastDetectedTagIDs = tempTagIDs;
        publishAprilTagsInRobotView.set(tempTagIDs);
        return averageTagDistance;
    }

    public void simulationPeriodicTesting() {
        int fiducialId = 8;
        Pose3d tagToCamPose = new Pose3d(-0.11740011460988099, -0.07212546513329927,0.5000514993226649,
                new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 0.0)));
        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        Optional<Pose3d> tagPose = currentTagFieldLayout == null ? Optional.empty() : currentTagFieldLayout.getTagPose(fiducialId);
        var targetPose = tagPose.get();


        Pose3d targetSpaceTop = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
        double z_top = Math.abs(targetSpaceTop.getZ());

        fiducialId = 0;
    }

    //ToDo - Check do we need any 3D pose of the robot wrt AprilTag
    public Pose2d getRobotPose() {
        return robotPose2DFieldSpaceRelativeToTeamOrigin;
    }

    //ToDo - Check do we need any 3D pose of the robot wrt AprilTag
    public Pose2d getDesiredRobotPose() {
        return this.desiredPose2d;
    }

     
     //Determing the desired pose to align to, with the x of the robot, y of the april tag,  and rotation toward the tag
     // i.e Try to align in front robot in front of April Tag
     // ToDo call this function from AutoBuilder.pathfindToPose i.e  Build a command to pathfind to a given pose     
    public Pose2d determineDesiredPose(Pose2d robotPose2d, Pose2d detectedTagIdPose2d) {
        double angle = Math.PI - detectedTagIdPose2d.getRotation().getRadians();
        desiredPose2d = new Pose2d(robotPose2d.getX(), detectedTagIdPose2d.getY(), new Rotation2d(angle));
        return desiredPose2d;
    }


    @Override
    public void periodic() {
        Pose2d desiredRobotPoseToAprilTag;

        //simulationPeriodicTesting();
        if (validVisionDetected()){

             LimeLightInstanceJsonResults = LimelightHelpers.getLatestResults(LimeLightInstance);
            AvgRobotDistanceFromTarget = this.GetAvgRobotDistanceFromTarget();
            // getRobotPose3D is measured from centre of the field by LL. Need offset correction 
            //botPose = getRobotPose3D();            

            if (selectedAlliance == Alliance.Red) {
                robotPose2DFieldSpaceRelativeToTeamOrigin = LimelightHelpers.getBotPose2d_wpiRed(LimeLightInstance);
                robotPose3DFieldSpaceRelativeToTeamOrigin = LimelightHelpers.getBotPose3d_wpiRed(LimeLightInstance);
            } else if(selectedAlliance == Alliance.Blue) {
                robotPose2DFieldSpaceRelativeToTeamOrigin = LimelightHelpers.getBotPose2d_wpiBlue(LimeLightInstance);
                robotPose3DFieldSpaceRelativeToTeamOrigin = LimelightHelpers.getBotPose3d_wpiBlue(LimeLightInstance);
            }

            //ToDo - If robot is in between 2 tags, how do we handle?
            Optional<Pose3d> tagPose = this.currentTagFieldLayout == null ? Optional.empty() : currentTagFieldLayout.getTagPose((int)this.lastDetectedTagIDs[0]);            
            desiredRobotPoseToAprilTag = this.determineDesiredPose(robotPose2DFieldSpaceRelativeToTeamOrigin, 
                                                             tagPose.get().toPose2d());

            //Log to Telemetry
            pubFieldPose3D.set(robotPose3DFieldSpaceRelativeToTeamOrigin);
            pubFieldPose3dArray.set(new Pose3d[] {this.robotPose3DFieldSpaceRelativeToTeamOrigin,
                                             this.getRobotPose3D()});
            pubEstimatedRobotPose2dArray.set(new Pose2d[] {robotPose2DFieldSpaceRelativeToTeamOrigin,
                                                           tagPose.get().toPose2d(),
                                                           desiredRobotPoseToAprilTag});
        }
        publishAprilTagDistance.set(AvgRobotDistanceFromTarget);
    }

}
