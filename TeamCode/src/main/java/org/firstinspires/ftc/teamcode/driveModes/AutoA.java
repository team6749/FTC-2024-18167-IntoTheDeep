package org.firstinspires.ftc.teamcode.driveModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotArm;


@Autonomous(name = "Auto-A")

public class AutoA extends OpMode {
    SampleMecanumDrive drive;
    RobotArm robotArm;
    private ElapsedTime mStateTime = new ElapsedTime();

    // todo: write your code here
//    SampleMecanumDrive drive = new SampleMecanumDrive();
    IMU imu2;
    int stepNbr;
    int END_STEP_NBR = 4;

    @Override
    public void init() {
//        robotArm = new RobotArm(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        stepNbr = 0;
    }

    @Override
    public void loop() {

        telemetry.addData("Step #", stepNbr);
        if (stepNbr <= END_STEP_NBR) {

            switch (stepNbr) {
                case 0:
                    // robotArm.driveMode()
//                    robotArm.driveMode();
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 1:
                    // Move forward 3 in
                    Trajectory fwd3 = drive.trajectoryBuilder(new Pose2d())
                            .forward(3)
                            .build();
                    drive.followTrajectory(fwd3);
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 2:
                    // Turn -90 degrees
                    drive.turn(Math.toRadians(90));
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 3:
                    // Left 3 ft
                    Trajectory sl36 = drive.trajectoryBuilder(new Pose2d())
                            .strafeLeft(36)
                            .build();
                    drive.followTrajectory(sl36);
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 4:
                    // Turn -45 degrees
                    drive.turn(Math.toRadians(45));
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 5:
                    //Forward-Right 3 in
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 6:
                    //robotArm.toHighBasket()
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 7:
                    //robotArm.openClaw()
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 8:
                    //robotArm.driveMode()
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 9:
                    //Rotate -45 degrees
                    stepNbr++;
                    mStateTime.reset();
                    break;
                case 10:
                    //Right 10-11 ft
                    stepNbr++;
                    mStateTime.reset();
                    break;
                default:
                    driveFieldRelative(0, 0, 0);
                    telemetry.addData("Auto", "Finished");
            }
        }
    }

    public void driveFieldRelative(double forward, double right, double rotate) {
//        double robotAngle = imu2.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        // convert to polar
//        double theta = Math.atan2(forward, right);
//        double r = Math.hypot(forward, right);
//        // rotate angle
//        theta = AngleUnit.normalizeRadians(theta - robotAngle);
//
//        // convert back to cartesian
//        double newForward = r * Math.sin(theta);
//        double newRight = r * Math.cos(theta);
//
//        drive.drive(newForward, newRight, rotate);
    }

}