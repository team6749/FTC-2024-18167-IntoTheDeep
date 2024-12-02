package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class OdometryTest extends LinearOpMode {
    public static double DISTANCE = 9.42; // in
    public Pose2d idealpose = new Pose2d(0.5,0);

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        waitForStart();
        drive.setDrivePower(idealpose);
        List<Integer> ticks = drive.getWheelTicks();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("encoderTicks",ticks);
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.addData("wheelVelocities",drive.getWheelVelocities());
//        telemetry.addData("wheelPositions",drive.getWheelPositions());
        telemetry.addData("wheelTicks",drive.getWheelTicks());

        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
