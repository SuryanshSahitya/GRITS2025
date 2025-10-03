// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.Pose;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.utils.SidePoseMatcher;

// public class Lynkalighnmentleft extends Command {
//   private final Drive s_Swerve;
//   private final boolean precise;
//   private final double speed;
//   private final PIDController xPID, yPID;
//   private final PIDController rotationPID = new PIDController(0.012, 0, 0);
//   private final double maxVisionDiff;
//   private final Timer alignedTimer = new Timer();
//   public static final double translationKP = 0.070;
//   public static final double roughTranslationKP = 0.10;
//   public static final double positionTolerance = 1.0; // inches
//   public static final double roughPositionTolerance = 2.5; // inches
//   public static final double positionKS = 0.02;
//   public static final double positionIZone = 4.0;
//   public static final double alignedTimerMax = 0.35;

//   public static final double rotationKP =
//       0.015; // Small overshoot at 0.015, more noticeable with 0.020, but still functional
//   public static final double rotationTolerance = 0.2; // degrees
//   public static final double roughRotatationTolerance = 1.5; // degrees

//   public Lynkalighnmentleft(
//       Drive s_Swerve, boolean flipIfRed, boolean precise, double speed, double maxVisionDiff) {
//     super();

//     this.s_Swerve = s_Swerve;
//     this.precise = precise;
//     this.speed = speed;
//     this.maxVisionDiff = maxVisionDiff;
//     addRequirements(s_Swerve);

//     xPID = new PIDController(precise ? 0.05 : 0.1, 0, 0);
//     yPID = new PIDController(precise ? 0.05 : 0.1, 0, 0);
//   }

//   private boolean isAligned() {
//     return Math.abs(xPID.getError()) <= xPID.getErrorTolerance()
//         && Math.abs(yPID.getError()) <= yPID.getErrorTolerance()
//         && Math.abs(rotationPID.getError()) <= rotationPID.getErrorTolerance();
//   }

//   @Override
//   public void initialize() {
//     super.initialize();

//     Pose2d targetPose = SidePoseMatcher.getBackedUpClosestLeftPosec(s_Swerve.getPose());

//     xPID.setIZone(positionIZone); // Only use Integral term within this range
//     xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
//     xPID.setSetpoint(Units.metersToInches(targetPose.getX()));
//     if (precise) {
//       xPID.setTolerance(positionTolerance, 5.0); // Inches per second
//     } else {
//       xPID.setTolerance(roughPositionTolerance);
//     }

//     yPID.setIZone(positionIZone); // Only use Integral term within this range
//     yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
//     yPID.setSetpoint(Units.metersToInches(targetPose.getY()));
//     if (precise) {
//       yPID.setTolerance(positionTolerance, 5.0); // Inches per second
//     } else {
//       yPID.setTolerance(roughPositionTolerance);
//     }

//     rotationPID.enableContinuousInput(-180.0, 180.0);
//     rotationPID.setIZone(Pose.rotationIZone); // Only use Integral term within this range
//     rotationPID.setIntegratorRange(-Pose.rotationKS * 2, Pose.rotationKS * 2);
//     rotationPID.setSetpoint(targetPose.getRotation().getDegrees());
//     if (precise) {
//       rotationPID.setTolerance(rotationTolerance, 10.0);
//     } else {
//       rotationPID.setTolerance(roughRotatationTolerance);
//     }

//     xPID.reset();
//     yPID.reset();
//     rotationPID.reset();
//     alignedTimer.stop();
//     alignedTimer.reset();

//     // Robot.field.getRobotObject().setTrajectory(targetPose);
//   }

//   @Override
//   public void execute() {
//     Pose2d pose = s_Swerve.getPose();
//     Translation2d position = pose.getTranslation();
//     Rotation2d rotation = pose.getRotation();

//     /* TODO Consider a potential need to rotate most of the way first, then translate */

//     double xCorrection = xPID.calculate(Units.metersToInches(position.getX()));
//     double xFeedForward = positionKS * Math.signum(xCorrection);
//     double xVal = MathUtil.clamp(xCorrection + xFeedForward, -1.0, 1.0);

//     double yCorrection = yPID.calculate(Units.metersToInches(position.getY()));
//     double yFeedForward = positionKS * Math.signum(yCorrection);
//     double yVal = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);

//     double correction = rotationPID.calculate(rotation.getDegrees());
//     double feedForward = Pose.rotationKS * Math.signum(correction);
//     double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

//     if (isAligned()) {
//       if (!alignedTimer.isRunning()) {
//         alignedTimer.restart();
//       }
//     } else if (alignedTimer.isRunning()) {
//       alignedTimer.stop();
//       alignedTimer.reset();
//     }

//     s_Swerve.drive3(
//         new Translation2d(xVal, yVal).times(speed),
//         rotationVal * Constants.MaxAngularRate.magnitude(),
//         precise);
//   }

//   // /* Drive */
//   // s_Swerve.drive(
//   //     // TODO Automatically go in turbo mode?
//   //     // new Translation2d(xVal, yVal).times((speed == PIDSpeed.FAST &&
//   // RobotState.getTurboMode()) ? PIDSpeed.TURBO.speed : speed.speed),
//   //     new Translation2d(xVal, yVal).times(5),
//   //     rotationVal * 5,
//   //     true
//   // );

//   @Override
//   public boolean isFinished() {
//     return s_Swerve.isAtTarget(
//         SidePoseMatcher.getBackedUpClosestLeftPosec(s_Swerve.getPose()), s_Swerve.getPose());
//   }
// }
