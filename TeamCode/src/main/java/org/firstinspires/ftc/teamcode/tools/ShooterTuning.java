package org.firstinspires.ftc.teamcode.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config // 必须加上这个注解，才能在 Dashboard 实时修改变量
@TeleOp(name = "Shooter PIDF 调优程序", group = "Debug")
public class ShooterTuning extends LinearOpMode {

    // 将参数定义为 public static，Dashboard 才能自动识别
    public static double TARGET_RPM = 0.0;
    public static double P = 0.00;
    public static double I = 0.0;      // 增加惯性轮后，I 可能有助于消除静差
    public static double D = 0.000;
    public static double F = 0.000;   // 前馈增益，维持转速的核心
    public static double TICKS_PER_REV = 28.0;

    private DcMotorEx SH, HS;
    private ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化 Dashboard 遥测
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SH = hardwareMap.get(DcMotorEx.class, "SH");
        HS = hardwareMap.get(DcMotorEx.class, "HS");

        SH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 维持原程序的电机方向
        SH.setDirection(DcMotorSimple.Direction.REVERSE);
        HS.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "等待开始...");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double currentVelTPS = SH.getVelocity();
            double targetVelTPS = (TARGET_RPM * TICKS_PER_REV) / 60.0;

            double dt = timer.seconds();
            timer.reset();
            if (dt == 0) dt = 1e-9;

            // PID 计算
            double error = targetVelTPS - currentVelTPS;

            // 积分项计算（带抗饱和逻辑：只有转速不为0时才积分）
            if (TARGET_RPM > 100) {
                integralSum += error * dt;
            } else {
                integralSum = 0;
            }

            // 限制积分范围，防止积分饱和（Windup）
            double maxIntegral = 0.25;
            if (integralSum > maxIntegral / I) integralSum = maxIntegral / I;
            if (integralSum < -maxIntegral / I) integralSum = -maxIntegral / I;

            double derivative = (error - lastError) / dt;
            lastError = error;

            // 总功率 = 前馈 + PID
            double power = (F * targetVelTPS) + (P * error) + (I * integralSum) + (D * derivative);

            // 安全限制
            if (TARGET_RPM <= 0) {
                power = 0;
                integralSum = 0;
            }

            SH.setPower(power);
            HS.setPower(power);

            // 数据观察 - 这里的 key 会在 Dashboard 的图表里显示
            double currentRPM = (currentVelTPS * 60.0) / TICKS_PER_REV;
            telemetry.addData("0-Target RPM", TARGET_RPM);
            telemetry.addData("1-Actual RPM", currentRPM);
            telemetry.addData("2-Error RPM", TARGET_RPM - currentRPM);
            telemetry.addData("3-Motor Power", power);
            telemetry.addData("4-Integral Sum", integralSum);
            telemetry.update();
        }
    }
}