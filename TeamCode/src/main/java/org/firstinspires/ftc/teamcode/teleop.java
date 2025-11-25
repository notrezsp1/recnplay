package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "teleop")
public class teleop extends OpMode {
    public DcMotor br, bl, shooter;
    public Servo servo;

    @Override
    public void init() {
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        shooter = hardwareMap.get(DcMotor.class, "claw");
        servo = hardwareMap.get(Servo.class, "servo");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }



    @Override
    public void loop() {
        // Controle do chassi
        double drive = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_y;
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        bl.setPower(leftPower);
        br.setPower(rightPower);


        if (gamepad1.right_trigger > 0.1) {
            shooter.setPower(1);
        } else if (gamepad1.left_trigger > 0.1) {
            shooter.setPower(-1);
        } else{
            shooter.setPower(0);
        }


        if (gamepad1.right_bumper){
            servo.setPosition(1);
        }else {
            servo.setPosition(0);
        }


    }
}