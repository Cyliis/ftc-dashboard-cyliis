package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Utils.PIDCoefficients;
import org.firstinspires.ftc.teamcode.Wrappers.CoolEncoder;
import org.firstinspires.ftc.teamcode.Wrappers.CoolMotor;

@Config
@TeleOp(name = "PID test")
public class PIDTest extends OpMode {

    public static PIDCoefficients pid = new PIDCoefficients(0,0,0);
    PIDController pidController = new PIDController(pid);
    CoolMotor motor1, motor2;
    CoolEncoder encoder;
    public static int target = 0;

    FtcDashboard dash;

    @Override
    public void init() {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        motor1 = new CoolMotor(hardwareMap.get(DcMotorEx.class,"motor1"), false);
        motor2 = new CoolMotor(hardwareMap.get(DcMotorEx.class,"motor2"), true);
        encoder = new CoolEncoder(hardwareMap.get(DcMotorEx.class,"motor1"));
    }

    @Override
    public void loop() {
        motor1.setPower(pidController.calculate(target - encoder.getCurrentPosition()));
        motor1.setPower(pidController.calculate(target - encoder.getCurrentPosition()));
        motor1.update();
        motor2.update();
        telemetry.addData("current", encoder.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.addData("integralSum", pidController.integralSum);
        telemetry.addData("temp", pidController.temp);
        telemetry.update();
    }
}
