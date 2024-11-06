package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotClaw {

    private Servo wristServo;
    private Servo leftClawServo;
    private Servo rightClawServo;

    public RobotClaw (HardwareMap hwMap){
        wristServo = hwMap.get(Servo.class,"wrist_servo");
        leftClawServo = hwMap.get(Servo.class,"left_claw_servo");
        rightClawServo = hwMap.get(Servo.class,"right_claw_servo");
    }
    public void openClaw() {

    }


    public void closeClaw() {

    }

    public boolean isOpen() {
        return true; //TODO actually check
    }

    public void toWristLeft() {

    }

    public void toWristCenter() {

    }

    public void toWristRight() {

    }
    private void turnWrist() {

    }
}
