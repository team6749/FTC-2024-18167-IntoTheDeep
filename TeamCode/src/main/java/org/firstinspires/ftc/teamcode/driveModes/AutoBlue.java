package org.firstinspires.ftc.teamcode.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name="Auto-Blue")

public class AutoBlue extends OpMode {
    private Blinker control_Hub;
    private DcMotor back_left;
    private DcMotor back_right;
    private Servo base_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private Servo gripper;
    private Gyroscope imu;
    private Servo middle_left;
    private Servo middle_right;
    private CRServo plane_shoot;
    private Servo wrist;
    private ElapsedTime mStateTime = new ElapsedTime();

    // todo: write your code here
    MecanumDrive drive = new MecanumDrive();
    IMU imu2;  
    int state;
    
    @Override
    public void init() {
        drive.init(hardwareMap);
        imu2 = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu2.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    
    @Override
    public void start() {
        state = 0;
    }

    @Override
    public void loop() {
        
        telemetry.addData("State",state);
        switch (state) {
            case 0:
                // move forward 10cm
                driveFieldRelative(0.1, 0, 0);
                state++;
                mStateTime.reset();
                break;
            case 1:
                // rotate 180
                if (mStateTime.time() >=1.0) {   
                    driveFieldRelative(0, 0, 0.2);
                    state++;
                    mStateTime.reset();
                }
                break;
            case 2:
                // rotate -180
                 if (mStateTime.time() >=1.0) {   
                    driveFieldRelative(0, 0, -0.2);
                    state++;
                    mStateTime.reset();
                 }
                break;
            case 3:
                // move back 10cm
                if (mStateTime.time() >=1.0) {   
                    driveFieldRelative(-0.1, 0, 0);
                    state++;
                    mStateTime.reset();
                }
                break;
            case 4:
                if (mStateTime.time() >=2.0 ) {
                    driveFieldRelative(0, 0, 0);
                    state++;
                    mStateTime.reset();
                }  
                break;
            default:
                driveFieldRelative(0, 0, 0);
                telemetry.addData("Auto","Finished");
        }
    }
    public void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu2.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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

}