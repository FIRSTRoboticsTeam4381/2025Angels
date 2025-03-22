package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public abstract class ConfidenceAlgorithm {

    public static ConfidenceAlgorithm[] algorithms;
    
    private SwerveDrivePoseEstimator poseEst;
    private StructPublisher<Pose2d> publisher;
    
    public ConfidenceAlgorithm(Swerve s, Pose2d startPose)
    {
        poseEst = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, s.getGyroYaw(), s.getPositions(), startPose);
        publisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/visiontests/"+getName(), Pose2d.struct).publish();
    }
    
    
    public void updateFromOdometry(Swerve s)
    {
        poseEst.update(s.getGyroYaw(), s.getPositions());

        publisher.set(poseEst.getEstimatedPosition());
    }

    public void updateFromVision(EstimatedRobotPose pose)
    {
        poseEst.addVisionMeasurement(pose.estimatedPose.toPose2d(),
         pose.timestampSeconds,
         getConfidence(pose));
    }

    public void resetRotation(Rotation2d r)
    {
        poseEst.resetRotation(r);
    }

    protected abstract String getName();

//private Matrix<N3, N1> confidenceMatrix = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 100, 100, 10000 }));


    protected abstract Matrix<N3, N1> getConfidence(EstimatedRobotPose pose);
}
