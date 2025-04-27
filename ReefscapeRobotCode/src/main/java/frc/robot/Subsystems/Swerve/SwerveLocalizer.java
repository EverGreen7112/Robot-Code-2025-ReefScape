
package frc.robot.Subsystems.Swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.LocalizationCamera;
import frc.robot.Utils.EverKit.Periodic;

public class SwerveLocalizer implements Periodic, SwerveConsts {
    private final boolean DEBUG_MODE = true;

    private static final LocalizationCamera[] CAMS = {
            new LocalizationCamera("left_cam",
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                    new Transform3d(new Translation3d(0.115, 0.055, 0.32), new Rotation3d(0, 0 ,0)),
                    VecBuilder.fill(0.0, 0.0, 0), VecBuilder.fill(0.0, 0.0, 0)),
            new LocalizationCamera("right_cam",
                                        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                                        new Transform3d(0.115, -0.155, 0.32, new Rotation3d(0, 0, 0)),
                                        VecBuilder.fill(0, 0, 0), VecBuilder.fill(0, 0, 0)),
            new LocalizationCamera("back_cam",
                                        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                                        new Transform3d(-0.27985, -0.295, 0.56, new Rotation3d(Math.toRadians(-1), Math.toRadians(-44), Math.toRadians(180))), //new Rotation3d(Math.toRadians(25), Math.toRadians(1.4), Math.toRadians(180))
                                        VecBuilder.fill(0, 0, 0), VecBuilder.fill(0, 0, 0))
    };

    private static final double FIELD_WIDTH = 8.05;
    private static final double FIELD_HEIGHT = 17.55;
    private static final double MAX_ESTIMATION_HEIGHT = 0.08;
    private static final double MAX_DISTANCE_FROM_TAG = 3;

    private static SwerveLocalizer m_instance = new SwerveLocalizer();
    private ArrayList<LocalizationCamera> m_cams;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private AprilTagFieldLayout m_fieldLayout;

    private SwerveLocalizer() {
        m_cams = new ArrayList<>(Arrays.asList(CAMS));
        m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(modulesPositions[0].x, modulesPositions[0].y),
                new Translation2d(modulesPositions[1].x, modulesPositions[1].y),
                new Translation2d(modulesPositions[2].x, modulesPositions[2].y),
                new Translation2d(modulesPositions[3].x, modulesPositions[3].y));

        m_poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                Swerve.getInstance().getGyroRotation2d(),
                Swerve.getInstance().getModulesPositions(),
                new Pose2d());

    }

    public static SwerveLocalizer getInstance() {
        return m_instance;
    }

    @Override
    public void periodic() {
        // update odometry
        m_poseEstimator.update(Swerve.getInstance().getGyroRotation2d(), Swerve.getInstance().getModulesPositions());

        // update vision
        for (LocalizationCamera cam : m_cams) {
            addCameraVisionMeasurements(cam);
        }
    }

    public Pose2d getCurrentPoint() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPoint(Pose2d newPoint) {
        m_poseEstimator.resetPosition(
                Swerve.getInstance().getGyroRotation2d(),
                Swerve.getInstance().getModulesPositions(),
                newPoint);
    }

    public double getFieldOrientedAngle() {
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    private boolean takeVisionPoseEstimation(Optional<EstimatedRobotPose> est) {
        if (!est.isPresent() || est == null)
            return false;

        Pose2d estPos = est.get().estimatedPose.toPose2d();
        double x = estPos.getX();
        double y = estPos.getY();
        double z = est.get().estimatedPose.getZ();

        boolean outOfField = x < 0.0 || x > FIELD_HEIGHT || y < 0.0 || y > FIELD_WIDTH;
        boolean aboveCamera = z > MAX_ESTIMATION_HEIGHT;
        boolean underGround = z < -0.03;

        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var tgt : est.get().targetsUsed) {
            var tagPose = m_fieldLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(est.get().estimatedPose.toPose2d().getTranslation());
        }
        avgDist /= numTags;

        boolean isTooFar = avgDist > MAX_DISTANCE_FROM_TAG;
        if(DEBUG_MODE){    
            SmartDashboard.putBoolean("out of field", outOfField);
            SmartDashboard.putBoolean("to far", isTooFar);
            SmartDashboard.putBoolean("above camera", aboveCamera);
            SmartDashboard.putBoolean("underGround", underGround);
        }
        return !outOfField && !aboveCamera && !underGround && !isTooFar;
    }

    private void addCameraVisionMeasurements(LocalizationCamera cam) {
        Optional<EstimatedRobotPose> est = cam.getEstimatedGlobalPose();
        if(DEBUG_MODE && est.isPresent())
            SmartDashboard.putString("vision pose", est.get().estimatedPose.toString());
        if (!takeVisionPoseEstimation(est))
            return;
        m_poseEstimator.addVisionMeasurement(est.get().estimatedPose.toPose2d(), est.get().timestampSeconds,
                cam.getEstimationStdDevs());
    }

    public boolean areCamsConnected(){
        for (LocalizationCamera cam : CAMS) {
            if(!cam.isConnected())
                return false;
        }
        return true;
    }

    public void initialize(){
        start(PeriodicTime.kRobotPeriodic);
    }

}
