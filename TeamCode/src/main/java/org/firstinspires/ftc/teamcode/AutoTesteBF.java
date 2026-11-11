package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
public class AutoTestBF extends LinearOpMode {
    // Nomes
    static final String M_LEF = "frontLeft";
    static final String M_LDF = "frontRight";
    static final String M_LET = "backLeft";
    static final String M_LDT = "backRight";
    static final String M_MT = "motorPotencia"; //lançador
    static final String M_MC = "motorExtra";
    static final String M_MC2 = "motorAuxiliar";
    static final String PINPOINT = "odo";

    static final String ServoGatilho = "ServoColetor";

    //Posição pods
    static final double xOffset = 95;
    static final double yOffset = 100;

    // Potências
    static final double POT      = 0.5;   // andando
    static final double POT_GIRO = 0.4;   // girando

    // Motores + Servo + Odo
    private DcMotor Lef, Ldf, Let, Ldt;
    private DcMotor Mt, Mc, Mc2;

    private CRServo Sg;
    private GoBildaPinpointDriver odo;
    static final GoBildaPinpointDriver.EncoderDirection DIR_ENC_X =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    static final GoBildaPinpointDriver.EncoderDirection DIR_ENC_Y =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    private double x, y, h;

    // =====================================================================
    @Override
    public void runOpMode() {
        config();

        waitForStart();
        if(isStopRequested()) return;

        trajeto();

        parar();
        desligarMecanismos();
    }

    public void config() {
        Lef = hardwareMap.get(DcMotor.class, M_LEF);
        Ldf = hardwareMap.get(DcMotor.class, M_LDF);
        Let = hardwareMap.get(DcMotor.class, M_LET);
        Ldt = hardwareMap.get(DcMotor.class, M_LDT);

        // Com estas direções: potência + nas 4 rodas = robô vai para FRENTE
        Lef.setDirection(DcMotor.Direction.REVERSE);
        Ldf.setDirection(DcMotor.Direction.FORWARD);
        Let.setDirection(DcMotor.Direction.REVERSE);
        Ldt.setDirection(DcMotor.Direction.FORWARD);

        Lef.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Ldf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Let.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Ldt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Mt = hardwareMap.get(DcMotor.class, M_MT);
        Mt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Mt.setDirection(DcMotor.Direction.FORWARD);
        Mt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Mc = hardwareMap.get(DcMotor.class, M_MC);
        Mc2 = hardwareMap.get(DcMotor.class, M_MC2);

        Mc.setDirection(DcMotor.Direction.FORWARD);
        Mc2.setDirection(DcMotor.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);
        odo.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(DIR_ENC_X, DIR_ENC_Y);
        odo.resetPosAndIMU();
        sleep(300);
        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
        atualizarOdo();

        Sg = hardwareMap.get(CRServo.class, ServoGatilho);
        Sg.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    //Servo coletor
    public void coletar(){
        Mc.setPower(1.0);
        Mc2.setPower(1.0);
    }

    public void atirar(){
        ElapsedTime cronometro = new ElapsedTime();
        cronometro.reset();

        while(opModeIsActive() && cronometro.seconds() < 4){
            Sg.setPower(1.0);
        }
        Sg.setPower(0);
    }
    public void lancar(){
        Mt.setPower(0.10);
    }
    public void atualizarOdo() {
        odo.update();
        Pose2D p = odo.getPosition();
        x = p.getX(DistanceUnit.MM);
        y = p.getY(DistanceUnit.MM);
        h = p.getHeading(AngleUnit.DEGREES);
    }

    public void gatilho(){
        Sg.setPower(1.0);
    }

    // =====================================================================
    // Movimentação: liga as rodas e desliga quando andou a distância
    // (ou girou o ângulo). Distância em mm, ângulo em graus.
    public void rodas(double lef, double ldf, double let, double ldt) {
        Lef.setPower(lef);
        Ldf.setPower(ldf);
        Let.setPower(let);
        Ldt.setPower(ldt);
    }

    public void andar(double distancia, double lef, double ldf, double let, double ldt) {
        atualizarOdo();
        double x0 = x, y0 = y;
        rodas(lef, ldf, let, ldt);
        while (opModeIsActive() && Math.hypot(x - x0, y - y0) < distancia) {
            atualizarOdo();
            tel();
        }
        parar();
    }

    public void girar(double angulo, double lef, double ldf, double let, double ldt) {
        atualizarOdo();
        double h0 = h;
        rodas(lef, ldf, let, ldt);
        while (opModeIsActive() && Math.abs(normalizar(h - h0)) < angulo) {
            atualizarOdo();
            tel();
        }
        parar();
    }

    private double normalizar(double graus) {
        while (graus > 180) graus -= 360;
        while (graus <= -180) graus += 360;
        return graus;
    }

    public void irFrente(double distancia){
        andar(distancia,  POT,  POT,  POT,  POT);
    }
    public void irTras(double distancia){
        andar(distancia, -POT, -POT, -POT, -POT);
    }
    public void irEsquerda(double distancia){
        andar(distancia,  POT, -POT, -POT,  POT);
    }
    public void irDireita(double distancia){
        andar(distancia, -POT,  POT,  POT, -POT);
    }

    public void girarDir(double angulo){
        girar(angulo,  POT_GIRO, -POT_GIRO,  POT_GIRO, -POT_GIRO);
    }
    public void girarEsq(double angulo){
        girar(angulo, -POT_GIRO,  POT_GIRO, -POT_GIRO,  POT_GIRO);
    }
    public void parar() {
        rodas(0, 0, 0, 0);
    }

    public void desligarMecanismos() {
        Mt.setPower(0);
        Mc.setPower(0);
        Mc2.setPower(0);
        Sg.setPower(0);
    }
    //Trajeto
    public void trajeto() {
        irFrente(1500.0);
        sleep(1000);
        girarEsq(120.0);
        sleep(1000);
        irTras(300);
        atirar();
        gatilho();
        irFrente(300);

        parar();

    }
    public void tel() {
        telemetry.addData("Pose", "%.0f %.0f %.0f", x, y, h);
        telemetry.update();
    }
}
