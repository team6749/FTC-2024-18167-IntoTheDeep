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
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(1);
    }


    public void closeClaw() {
        leftClawServo.setPosition(1);
        rightClawServo.setPosition(0);
    }

    public boolean isOpen() {
        //If the servers are close to the fully extended position, count as open
        return (leftClawServo.getPosition() < .2 ||
                rightClawServo.getPosition() > .8);
    }

    public void toWristLeft() {
    wristServo.setPosition(0.75); //DO NOT SET to 0 because it hits other things
    }

    public void toWristCenter() {
    wristServo.setPosition(0.5);
    }

    public void toWristRight() {
    wristServo.setPosition(.25);
    }
    public double getWristPosition() {
        return wristServo.getPosition();
    }
}
