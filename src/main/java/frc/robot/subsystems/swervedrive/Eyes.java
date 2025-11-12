/*
 * This subsytem controls robot vision. It uses the LimelightHelpers class
 * and contains methods that are used to track april tags and gather
 * positional data from them. This data is then used in the swerve
 * subsystem to update the robot's odometry using the pose estimator.
 * 
 * parameters:
 * none
 */


 package frc.robot.subsystems.swervedrive;


 import frc.robot.Constants;
 import java.util.List;
 
 import com.pathplanner.lib.path.GoalEndState;
 import com.pathplanner.lib.path.IdealStartingState;
 import com.pathplanner.lib.path.PathConstraints;
 import com.pathplanner.lib.path.PathPlannerPath;
 import com.pathplanner.lib.path.Waypoint;
 
 import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.DriverStation.Alliance;
 import edu.wpi.first.wpilibj.Timer;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.LimelightHelpers;
 import frc.robot.LimelightHelpers.LimelightResults;
 import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.util.Units;
 import edu.wpi.first.networktables.NetworkTable;
 import edu.wpi.first.networktables.NetworkTableEntry;
 import edu.wpi.first.networktables.NetworkTableInstance;
 import edu.wpi.first.networktables.StructPublisher;
 import frc.robot.subsystems.swervedrive.*;
 
 
 
 public class Eyes extends SubsystemBase {
 
     // Swerve subsystem for pose estimator
     SwerveSubsystem s_Swerve;
     LimelightHelpers r_LimelightHelpers;
 
     // create objects and variables
     public LimelightHelpers limelight;
     public double tx;
     public double ty;
     public double ta;
     public double tID;
     public double txnc;
     public double tync;
 
     public int closestReefSide;
     private StructPublisher<Pose2d> lReefPose = NetworkTableInstance.getDefault()
         .getStructTopic("Left Reef Pose", Pose2d.struct).publish();
     private StructPublisher <Pose2d> rReefPose = NetworkTableInstance.getDefault()
         .getStructTopic("Right Reef Pose", Pose2d.struct).publish();
 
 
    
     public boolean controllerRumble = false;
     public PathPlannerPath reefPath;
     public boolean closeToReef = false;
     public double closestDistance;
     // constuctor
     public Eyes(SwerveSubsystem swerve) {
 
         s_Swerve = swerve;
         // reefPath = closestReefpath(-1);
         
     }
 
  
     /*
      * This method will gather all of the positional data of the limelight target.
      * 
      * parameters:
      * none
      * 
      * returns;
      * none
      */
     public void updateData() {
 
         /* 
          * get data from limelight target
          * tx = x position in degrees in limelight FOV
          * ty = y position in degrees in limelight FOV
          * ta = pitch in degrees in limelight FOV
          * tID = target ID number
          */
         NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
         NetworkTableEntry tx = table.getEntry("tx");
         NetworkTableEntry ty = table.getEntry("ty");
         NetworkTableEntry ta = table.getEntry("ta");
 
         //read values periodically
         double x = tx.getDouble(tID);
         double y = ty.getDouble(0.0);
         double area = ta.getDouble(0.0);
 
         //post to smart dashboard periodically
         SmartDashboard.putNumber("LimelightX", x);
         SmartDashboard.putNumber("LimelightY", y);
         SmartDashboard.putNumber("LimelightArea", area);
 
         // tx = LimelightHelpers.getTX("limelight");
         // ty = LimelightHelpers.getTY("limelight");
         // ta = LimelightHelpers.getTA("limelight");
         // tID = LimelightHelpers.getFiducialID("limelight");
 
         // txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
         // tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees
 
         LimelightHelpers.setPipelineIndex("limelight-front", 0);
 
         // log target data
         SmartDashboard.putNumber("AprilTagID", tID);
 
     }
 
     /*
      * This method will wrap all target data into an array for easy access.
      * 
      * Array indexes:
      * [0] = x
      * [1] = y
      * [2] = a (pitch)
      * [3] = ID
      */
     public double[] getDataPackage() {
 
         double[] data = {
             tx,
             ty,
             ta,
             tID
         };
 
         return data;
     }
 
     /*
      * This method will return the pose of the robot based upon the pose of a detected apriltag
      * 
      * parameters:
      * none
      * 
      * returns:
      * robot pose      (Pose2d)
      */
     public Pose2d getRobotPose() {
 
         Pose2d pose;
 
         
         pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-front");
         return pose;
         
         
     }
     
 
     /*
      * This method will return the known pose of the desired target.
      * 
      * parameters:
      * none
      * 
      * returns:
      * target pose      (Pose2d)
      */
     
 
     @Override
     public void periodic() {

         if (LimelightHelpers.getTV("limelight-front") == true) {
             s_Swerve.getSwerveDrive().addVisionMeasurement(
                 getRobotPose(), 
                 Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight-front")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight")/1000.0)
             );
         }
 
         updateData();
 
        
 
 
        
 
     }
 }