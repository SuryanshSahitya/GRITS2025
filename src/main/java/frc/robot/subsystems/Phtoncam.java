package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class Phtoncam extends SubsystemBase {

  // Map which camera index is physically LEFT vs RIGHT on robot.
  // Swap these if your wiring/naming is opposite.
  private static final int LEFT_CAM_INDEX = 0; // index of left-mounted camera
  private static final int RIGHT_CAM_INDEX = 1; // index of right-mounted camera
  private static final int DEFAULT_CAM_INDEX = RIGHT_CAM_INDEX; // fallback choice

  /** Small wrapper to keep each camera’s objects together. */
  private static class VisionCam {
    final String name;
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;
    double lastEstTimestamp = 0.0;

    // Simulation-only
    PhotonCameraSim cameraSim;

    VisionCam(String name, Transform3d robotToCam, AprilTagFieldLayout layout) {
      this.name = name;
      this.camera = new PhotonCamera(name);
      this.estimator =
          new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }
  }

  private final Drive drivebase;
  private final AprilTagFieldLayout fieldLayout;
  private final List<VisionCam> cams = new ArrayList<>();

  // Simulation
  private VisionSystemSim visionSim;

  public Phtoncam(Drive drivebase) {
    this.drivebase = drivebase;

    // Load field once for all cameras
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Define camera transforms (TUNE to your robot mounts)
    // Example: cameras at ±6" Y offset, same height/pitch, slight outward yaw.
    Transform3d kRobotToLeftCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.0),
                Units.inchesToMeters(+6.0),
                Units.inchesToMeters(30.625)),
            new Rotation3d(0.0, Math.toRadians(35.0), Math.toRadians(+10.0)));

    Transform3d kRobotToRightCam =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.0),
                Units.inchesToMeters(-6.0),
                Units.inchesToMeters(30.625)),
            new Rotation3d(0.0, Math.toRadians(35.0), Math.toRadians(-10.0)));

    // Create cameras (names must match PhotonVision config)
    cams.add(new VisionCam("Cam1left", kRobotToLeftCam, fieldLayout)); // index 0
    cams.add(new VisionCam("Cam2right ", kRobotToRightCam, fieldLayout)); // index 1

    // Simulation setup
    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");

      var camProps = new SimCameraProperties();
      camProps.setCalibration(960, 720, Rotation2d.fromDegrees(90));
      camProps.setCalibError(0.35, 0.10);
      camProps.setFPS(15);
      camProps.setAvgLatencyMs(50);
      camProps.setLatencyStdDevMs(15);

      cams.get(LEFT_CAM_INDEX).cameraSim =
          new PhotonCameraSim(cams.get(LEFT_CAM_INDEX).camera, camProps);
      cams.get(RIGHT_CAM_INDEX).cameraSim =
          new PhotonCameraSim(cams.get(RIGHT_CAM_INDEX).camera, camProps);

      visionSim.addCamera(cams.get(LEFT_CAM_INDEX).cameraSim, kRobotToLeftCam);
      visionSim.addCamera(cams.get(RIGHT_CAM_INDEX).cameraSim, kRobotToRightCam);

      cams.get(LEFT_CAM_INDEX).cameraSim.enableDrawWireframe(true);
      cams.get(RIGHT_CAM_INDEX).cameraSim.enableDrawWireframe(true);
    }
  }

  // Only fuse measurement from the camera selected by auto-align side
  // If side == "left"  -> use RIGHT camera for fusion
  // If side == "right" -> use LEFT camera  for fusion
  @Override
  public void periodic() {
    int camIndex = getFusionCamIndex();

    var visionEst = getEstimatedGlobalPose(camIndex);
    visionEst.ifPresent(
        est -> {
          var estPose = est.estimatedPose.toPose2d();
          var estStdDevs = getEstimationStdDevs(estPose, camIndex);

          // Fuse a single measurement from the selected camera only
          drivebase.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);

          if (Robot.isSimulation()) {
            // Draw chosen camera's estimate
            getSimDebugField()
                .getObject("VisionEstimation_" + cams.get(camIndex).name)
                .setPose(estPose);

            // Clear other camera to emphasize it's not fused this cycle
            int other = (camIndex == LEFT_CAM_INDEX) ? RIGHT_CAM_INDEX : LEFT_CAM_INDEX;
            getSimDebugField().getObject("VisionEstimation_" + cams.get(other).name).setPoses();
          }
        });
  }

  /** Decide which camera to fuse based on your side state. */
  public int getFusionCamIndex() {
    String side = frc.robot.Constants.getautoalignside().toString(); // expected "left" or "right"
    if (side == null) return DEFAULT_CAM_INDEX;

    if ("left".equalsIgnoreCase(side)) {
      return RIGHT_CAM_INDEX; // align with opposite camera
    } else if ("right".equalsIgnoreCase(side)) {
      return LEFT_CAM_INDEX;
    }
    return DEFAULT_CAM_INDEX;
  }

  /** Latest pipeline result from the currently-selected alignment camera. */
  public PhotonPipelineResult getAutoAlignLatestResult() {
    return getLatestResult(getFusionCamIndex());
  }

  /** Latest estimated pose from the currently-selected alignment camera. */
  public Optional<EstimatedRobotPose> getAutoAlignEstimatedPose() {
    return getEstimatedGlobalPose(getFusionCamIndex());
  }

  /** Back-compat: returns the LEFT camera (index 0) result. */
  public PhotonPipelineResult getLatestResult() {
    return cams.get(LEFT_CAM_INDEX).camera.getLatestResult();
  }

  /** Returns pipeline result for specific camera index. */
  public PhotonPipelineResult getLatestResult(int camIndex) {
    return cams.get(camIndex).camera.getLatestResult();
  }

  /** Back-compat: estimated robot pose from LEFT camera. */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return getEstimatedGlobalPose(LEFT_CAM_INDEX);
  }

  /** Estimated robot pose from a specific camera index. */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(int camIndex) {
    VisionCam vc = cams.get(camIndex);
    var visionEst = vc.estimator.update(vc.camera.getLatestResult());
    double latestTimestamp = vc.camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - vc.lastEstTimestamp) > 1e-5;

    if (Robot.isSimulation()) {
      visionEst.ifPresentOrElse(
          est ->
              getSimDebugField()
                  .getObject("VisionEstimation_" + vc.name)
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            if (newResult) getSimDebugField().getObject("VisionEstimation_" + vc.name).setPoses();
          });
    }
    if (newResult) vc.lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /** Std devs for SwerveDrivePoseEstimator (back-compat = LEFT cam). */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    return getEstimationStdDevs(estimatedPose, LEFT_CAM_INDEX);
  }

  /** Std devs for a specific camera, scaled by tag count and distance. */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, int camIndex) {
    var estStdDevs = VecBuilder.fill(0.5, 0.5, 1.0);

    var targets = getLatestResult(camIndex).getTargets();
    int numTags = 0;
    double avgDist = 0.0;

    for (var tgt : targets) {
      var tagPose = cams.get(camIndex).estimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;

    avgDist /= numTags;

    // Decrease std devs if multiple tags visible
    if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 1.0);

    // If only one tag and it's far, effectively reject
    if (numTags == 1 && avgDist > 4.0) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30.0));
    }

    return estStdDevs;
  }

  // Simulation hooks

  public void simulationPeriodic(Pose2d robotSimPose) {
    if (visionSim != null) {
      visionSim.update(robotSimPose);
    }
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation() && visionSim != null) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation() || visionSim == null) return null;
    return visionSim.getDebugField();
  }
}
