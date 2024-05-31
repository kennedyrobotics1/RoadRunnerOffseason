package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (group = "drive")
public class FinalPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //starts one tile forward from bottom right corner, angled 90 degrees to the left
        drive.setPoseEstimate(new Pose2d(-30, -50, Math.toRadians(90)));

        Trajectory lineToTopRightCorner = drive.trajectoryBuilder(new Pose2d(-30, -50, Math.toRadians(90)))
                .lineTo(new Vector2d(59, -47))
                .build();
        Trajectory lineToTopLeftCornerAndRotation = drive.trajectoryBuilder(lineToTopRightCorner.end())
                .lineToLinearHeading(new Pose2d(55, 65, Math.toRadians(180)))
                .build();

        //2nd path

        //starts at middle left side of field
        Trajectory splineToMiddleLeft = drive.trajectoryBuilder(new Pose2d(-40, 15, Math.toRadians(0)))
                .splineTo(new Vector2d(-5,30), Math.toRadians(90))
                .build();
        Trajectory lineToFarLeft = drive.trajectoryBuilder(traj0.end())
                .lineToLinearHeading(new Pose2d(-5,35, Math.toRadians(0)))
                .build();
        Trajectory splineToTopLeftCorner = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(33, 45), Math.toRadians(270))
                .build();
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(splineToMiddleLeft);
        drive.followTrajectory(lineToFarLeft);
        drive.followTrajectory(splineToTopLeftCorner);



        //starts at top left corner
        Trajectory forwardToTopRightCorner = drive.trajectoryBuilder(new Pose2d(50, 50, Math.toRadians(270)))
                        .lineTo(new Vector2d(50, -50))
                        .build();

        Trajectory rotateToMiddleRight = drive.trajectoryBuilder(new Pose2d(50, -50, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-30, -50, Math.toRadians(90)))
                        .build();

        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(lineToTopRightCorner);
        drive.followTrajectory(lineToTopLeftCornerAndRotation);



        drive.followTrajectory(forwardToTopRightCorner);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(rotateToMiddleRight);
    }
}
