// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class Strykeforce2024 extends ConfidenceAlgorithm {

    public Strykeforce2024(Swerve s, Pose2d start) {
        super(s, start);
    }

    @Override
    protected String getName() {
        return "Stykeforce2024";
    }

    @Override
    protected Matrix<N3, N1> getConfidence(EstimatedRobotPose pose) {
        double minDistance = 2767;
        double avg = 0.0;

        for (PhotonTrackedTarget t : pose.targetsUsed) {
            double d = t.bestCameraToTarget.getTranslation().getNorm();
            avg += d * 1.0 / pose.targetsUsed.size();
            if (d < minDistance) {
                minDistance = d;
            }
        }
       
           double xyStdDev= 
            1 / Math.pow(
            Math.E,
                Math.pow(
                     17.0 / 100.0 * avg,
                     4.0));
                     return new Matrix<N3,N1>(new SimpleMatrix(new double[]{xyStdDev,xyStdDev,10000}));
        }
    }

