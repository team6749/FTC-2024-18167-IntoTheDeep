package org.firstinspires.ftc.teamcode.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotArm;
import org.firstinspires.ftc.teamcode.hardware.RobotClaw;
import org.firstinspires.ftc.teamcode.subsystems.CraneSystem;

@Disabled
@TeleOp(name="TeleOp: Field-Oriented", group="Iterative OpMode")
public class FieldRelativeMecanumDriveOpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    CraneSystem craneSystem;
    RobotClaw robotClaw;
    IMU imu;

    @Override
    public void init() {
        drive.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
//        craneSystem = new CraneSystem(null, new RobotArm(hardwareMap));
        robotClaw = new RobotClaw(hardwareMap);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    public void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
    }


    @Override
    public void loop() {
//        double forward = -gamepad1.left_stick_y/3;
//        double right = gamepad1.left_stick_x/3;
//        double rotate = gamepad1.right_stick_x/3;
//        // if right trigger, turbo mode
//        if (gamepad1.right_bumper){
//            driveFieldRelative(forward*2, right*2, rotate);
//        } else {
//            driveFieldRelative(forward, right, rotate);
//        }
//
//        if (gamepad1.right_trigger > 0) {
//            craneSystem.dumpAtHighBasket(gamepad1.right_trigger);
//            System.out.println(gamepad1.right_trigger);
//        } else if (gamepad1.left_trigger > 0) {
//            craneSystem.backToBase(gamepad1.left_trigger);
//        } else {
//            craneSystem.stop();
//        }
//
//        //Claw Movement
//        if (gamepad1.x) {
//            robotClaw.openClaw();
//        }
//        if (gamepad1.y){
//            robotClaw.closeClaw();
//        }
//
//
//        // Wrist movement
//        if (gamepad1.a){// && !craneSystem.IS_IN_DANGER_ZONE){
//            robotClaw.toWristLeft();
//            telemetry.addData("Wrist", "left");
//
//        } else if (gamepad1.b){// && !craneSystem.IS_IN_DANGER_ZONE){
//            robotClaw.toWristRight();
//            telemetry.addData("Wrist", "right");
//
//        } else {
//            robotClaw.toWristCenter();
//        }
//
//        telemetry.addData("Extender Motor enc", craneSystem.getCurrentExtension());
//        telemetry.addData("Lift Motor enc", craneSystem.getCurrentRotation());
    }
}