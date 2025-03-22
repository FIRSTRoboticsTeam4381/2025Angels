// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class TorBots extends ConfidenceAlgorithm {

    public TorBots(Swerve s, Pose2d start) {
        super(s, start);
    }

    @Override
    protected String getName() {
        return "TorBots";
    }

    @Override
    protected Matrix<N3, N1> getConfidence(EstimatedRobotPose pose) {
        double lowest_dist = Double.MAX_VALUE;
        double avg_dist = 0;

        for (PhotonTrackedTarget t : pose.targetsUsed) {
            double d = t.bestCameraToTarget.getTranslation().getNorm();
            avg_dist += d;
            if (d < lowest_dist)
                lowest_dist = d;
        }

        double xyStdDev = 4.5 * ((0.01 + (0.08 * Math.pow(avg_dist, 2.0))) / pose.targetsUsed.size());

        SmartDashboard.putNumber("avg dist", avg_dist);
        SmartDashboard.putNumber("xyStdDev", xyStdDev);
        xyStdDev = Math.max(0.03, xyStdDev);

        return new Matrix<N3, N1>(new SimpleMatrix(new double[] { xyStdDev, xyStdDev, 10000 }));

    }
}
