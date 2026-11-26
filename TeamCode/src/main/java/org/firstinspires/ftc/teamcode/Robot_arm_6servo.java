package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Robot Arm Test 6 servo")
public class Robot_arm_6servo extends LinearOpMode {

    private Servo s1,s2,s3,s4,s5,s6;
    @Override
    public void runOpMode() throws InterruptedException {
        s1=hardwareMap.get(Servo.class,"s1");
        s2=hardwareMap.get(Servo.class,"s2");
        s3=hardwareMap.get(Servo.class,"s3");
        s4=hardwareMap.get(Servo.class,"s4");
        s5=hardwareMap.get(Servo.class,"s5");
        s6=hardwareMap.get(Servo.class,"s6");
        s1.setPosition(0);
        s2.setPosition(0);
        s3.setPosition(0);
        s4.setPosition(0);
        s5.setPosition(0);
        s6.setPosition(0);
        waitForStart();
        while (opModeIsActive()){
            s1.setPosition(0.5);
            s2.setPosition(0.5);
            s3.setPosition(0.5);
            s4.setPosition(0.5);
            s5.setPosition(0.5);
            s6.setPosition(0.5);

        }
//        s1.setPosition(0);
//        s2.setPosition(0);
//        s3.setPosition(0);
//        s4.setPosition(0);
//        s5.setPosition(0);
//        s6.setPosition(0);
    }
}
