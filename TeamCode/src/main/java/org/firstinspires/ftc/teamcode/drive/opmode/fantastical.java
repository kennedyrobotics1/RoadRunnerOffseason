package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;


@Autonomous (group = "drive")
public class fantastical extends LinearOpMode {

    boolean style = false;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /*Trajectory goattraj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(35, -35), Math.toRadians(90))
                .splineTo(new Vector2d(30, 35), Math.toRadians(180))
                .splineTo(new Vector2d(-35, 35), Math.toRadians(270))
                .splineTo(new Vector2d(-35, -35), Math.toRadians(45))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();
        Trajectory goattraj2 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(-45)))
                        .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(90)))
                                .build();
        Trajectory goattraj3 = drive.trajectoryBuilder(new Pose2d(35, -35, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(30, 35, Math.toRadians(180)))
                                .build();
        Trajectory goattraj4 = drive.trajectoryBuilder(goattraj3.end())
                        .lineToLinearHeading(new Pose2d(-38, 38, Math.toRadians(180)))
                                .build();
        Trajectory goattraj5 = drive.trajectoryBuilder(goattraj4.end())
                        .lineTo(new Vector2d(-38, -38))

                                .build();
        Trajectory goattraj6 = drive.trajectoryBuilder(goattraj5.end())
                        .lineTo(new Vector2d(0, 0))
                .addDisplacementMarker(() -> {
                    style = true;
        })
                                .build();


         */

        drive.setPoseEstimate(new Pose2d(-30, -50, Math.toRadians(90)));

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-30, -50, Math.toRadians(90)))
                        .lineTo(new Vector2d(59, -47))
                                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .lineToLinearHeading(new Pose2d(55, 65, Math.toRadians(180)))
                                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);


    }
}
