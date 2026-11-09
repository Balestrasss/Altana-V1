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

    //Posição pods (ATENÇÃO: no TeleOp está 30 e 12 — um dos dois está errado, medir!)
    static final double xOffset = 95;
    static final double yOffset = 100;

    //Tolerancia
    static final double tol  = 15;      // mm
    static final double tolH  = 4;      // graus

    // Controle do movimento
    static final double VEL     = 0.5;    // potência máxima andando
    static final double POT_MIN = 0.12;   // mínimo para vencer o atrito (senão para antes do ponto)
    static final double KP      = 0.004;  // potência por mm de erro
    static final double KP_H    = 0.02;   // potência por grau de erro
    static final double TIMEOUT = 5.0;    // s: se não chegar, desiste (bateu em algo)

    // Sinais: troque para -1 o que o MODO TESTE mandar (ver tela do teste)
    static final double SINAL_LATERAL = 1;
    static final double SINAL_GIRO    = 1;

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
    private double erX, erY, erH;
    private boolean modoTeste = false;
    private boolean abortado = false;
    private String aviso = "";

    // =====================================================================
    @Override
    public void runOpMode() {
        config();

        boolean yAnterior = false;
        while (opModeInInit()) {        // mostra a pose no INIT (confere se o Pinpoint responde)
            if (gamepad1.y && !yAnterior) modoTeste = !modoTeste;   // Y no INIT = liga/desliga o teste
            yAnterior = gamepad1.y;
            atualizarOdo();
            telemetry.addData("MODO TESTE (Y no gamepad1)", modoTeste ? "LIGADO" : "desligado");
            tel();
        }
        if(isStopRequested()) return;

        if (modoTeste) {
            testeDirecoes();
            return;
        }

        trajeto();

        parar();
        desligarMecanismos();

        while (opModeIsActive()) {
            atualizarOdo();
            tel();
        }
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
        // RUN_USING_ENCODER só se o cabo de encoder do lançador estiver ligado.
        // Sem o cabo, o controle de velocidade do Hub "acha" que o motor está parado e manda tudo.
        Mt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Mt.setDirection(DcMotor.Direction.FORWARD);
        Mt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // igual ao TeleOp

        Mc = hardwareMap.get(DcMotor.class, M_MC);
        Mc2 = hardwareMap.get(DcMotor.class, M_MC2);

        Mc.setDirection(DcMotor.Direction.FORWARD);
        Mc2.setDirection(DcMotor.Direction.REVERSE);   // igual ao TeleOp (Auxilio = REVERSE)

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
            atualizarOdo();
            tel();
        }
        Sg.setPower(0);
    }
    public void lancar(){
        Mt.setPower(0.10);      // TeleOp usa até 0.54; 0.10 quase não gira
    }
    public void atualizarOdo() {
        odo.update();
        Pose2D p = odo.getPosition();
        x = p.getX(DistanceUnit.MM);     // X do Pinpoint = FRENTE
        y = p.getY(DistanceUnit.MM);     // Y do Pinpoint = ESQUERDA
        h = p.getHeading(AngleUnit.DEGREES);   // + = girou para a ESQUERDA
    }

    public void gatilho(){
        Sg.setPower(1.0);
    }

    // =====================================================================
    // Base: mesma mistura do TeleOp.
    // axial + = frente, lateral + = direita, rot + = girar p/ direita
    public void mover(double axial, double lateral, double rot) {
        lateral *= SINAL_LATERAL;
        rot     *= SINAL_GIRO;
        double vLef = axial - lateral + rot;
        double vLdf = axial + lateral - rot;
        double vLet = axial + lateral + rot;
        double vLdt = axial - lateral - rot;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(vLef), Math.abs(vLdf)),
                Math.max(Math.abs(vLet), Math.abs(vLdt))));
        Lef.setPower(vLef / max);
        Ldf.setPower(vLdf / max);
        Let.setPower(vLet / max);
        Ldt.setPower(vLdt / max);
    }

    /**
     * Vai até a posição (x, y em mm; ângulo em graus) ABSOLUTA do campo,
     * lendo a odometria a CADA ciclo. Para quando chega, estoura o tempo
     * ou o juiz aperta STOP.
     */
    public void irPara(double alvoX, double alvoY, double alvoH) {
        if (abortado) return;
        ElapsedTime t = new ElapsedTime();
        atualizarOdo();
        double erroAngInicial = Math.abs(normalizar(alvoH - h));
        while (opModeIsActive() && t.seconds() < TIMEOUT) {
            atualizarOdo();

            erX = alvoX - x;
            erY = alvoY - y;
            erH = normalizar(alvoH - h);
            double dist = Math.hypot(erX, erY);

            // TRAVA: o erro de ângulo CRESCEU muito = o giro está corrigindo para o lado errado
            if (Math.abs(erH) > erroAngInicial + 30) {
                parar();
                abortado = true;
                aviso = "PAROU: o ângulo foi para o lado errado. Rode o MODO TESTE (Y no INIT).";
                tel();
                return;
            }

            if (dist < tol && Math.abs(erH) < tolH) break;   // chegou

            // erro do campo -> erro do ponto de vista do robô
            double hr = Math.toRadians(h);
            double frente   =  erX * Math.cos(hr) + erY * Math.sin(hr);
            double esquerda = -erX * Math.sin(hr) + erY * Math.cos(hr);

            // translação proporcional à distância, com mínimo e máximo
            double vel = Math.min(VEL, Math.max(POT_MIN, KP * dist));
            if (dist < tol) vel = 0;
            double axial   = dist > 0 ? frente   / dist * vel : 0;
            double lateral = dist > 0 ? -esquerda / dist * vel : 0;

            // giro proporcional ao erro de ângulo (erH + = precisa ir p/ esquerda)
            double rot = -Math.max(-VEL, Math.min(VEL, KP_H * erH));
            if (Math.abs(erH) >= tolH && Math.abs(rot) < POT_MIN) rot = -Math.signum(erH) * POT_MIN;

            mover(axial, lateral, rot);
            tel();
        }
        parar();
    }

    private double normalizar(double graus) {
        while (graus > 180) graus -= 360;
        while (graus <= -180) graus += 360;
        return graus;
    }

    // Movimentação (distâncias RELATIVAS à posição atual do robô, mantendo o ângulo)
    public void irFrente(double distancia){
        atualizarOdo();
        double hr = Math.toRadians(h);
        irPara(x + distancia * Math.cos(hr), y + distancia * Math.sin(hr), h);
    }
    public void irTras(double distancia){
        irFrente(-distancia);
    }
    public void irEsquerda(double distancia){
        atualizarOdo();
        double hr = Math.toRadians(h);
        irPara(x - distancia * Math.sin(hr), y + distancia * Math.cos(hr), h);
    }
    public void irDireita(double distancia){
        irEsquerda(-distancia);
    }

    public void girarDir(double angulo){
        atualizarOdo();
        irPara(x, y, normalizar(h - angulo));
    }
    public void girarEsq(double angulo){
        atualizarOdo();
        irPara(x, y, normalizar(h + angulo));
    }
    public void parar() {
        Lef.setPower(0);
        Ldf.setPower(0);
        Let.setPower(0);
        Ldt.setPower(0);
    }
    /**
     * MODO TESTE: anda um pouco em cada direção e diz qual sinal trocar.
     * Deixe ~1 m livre em volta do robô.
     */
    public void testeDirecoes() {
        double[] f = testar(0.3, 0, 0);     // frente
        double[] l = testar(0, 0.3, 0);     // direita
        double[] g = testar(0, 0, 0.3);     // girar p/ direita

        String frente = (f[0] > 50 && Math.abs(f[2]) < 15) ? "OK"
                : (f[0] < -50 ? "ANDOU P/ TRÁS: inverta as 4 direções das rodas no config()"
                   : "GIROU/ANDOU TORTO: direções das rodas esquerda x direita erradas no config()");
        String lado = l[1] < -50 ? "OK" : (l[1] > 50 ? "INVERTIDO: SINAL_LATERAL = -1" : "não andou de lado: confira as rodas");
        String giro = g[2] < -10 ? "OK" : (g[2] > 10 ? "INVERTIDO: SINAL_GIRO = -1" : "não girou: confira as rodas");

        while (opModeIsActive()) {
            telemetry.addLine("=== RESULTADO DO TESTE ===");
            telemetry.addData("Frente", "%s  (dx=%.0f dy=%.0f dh=%.0f)", frente, f[0], f[1], f[2]);
            telemetry.addData("Direita", "%s  (dx=%.0f dy=%.0f dh=%.0f)", lado, l[0], l[1], l[2]);
            telemetry.addData("Giro dir.", "%s  (dx=%.0f dy=%.0f dh=%.0f)", giro, g[0], g[1], g[2]);
            telemetry.addData("Pinpoint", odo.getDeviceStatus());
            telemetry.update();
        }
    }

    /** Manda (axial, lateral, rot) por 0.6 s e devolve {dx, dy, dh} medidos (dx/dy do ponto de vista do robô). */
    private double[] testar(double axial, double lateral, double rot) {
        sleep(400);
        atualizarOdo();
        double x0 = x, y0 = y, h0 = h;
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < 0.6) {
            mover(axial, lateral, rot);
            atualizarOdo();
        }
        parar();
        sleep(400);
        atualizarOdo();
        double hr = Math.toRadians(h0);
        double dxc = x - x0, dyc = y - y0;
        return new double[]{dxc * Math.cos(hr) + dyc * Math.sin(hr),
                -dxc * Math.sin(hr) + dyc * Math.cos(hr),
                normalizar(h - h0)};
    }

    public void desligarMecanismos() {
        Mt.setPower(0);
        Mc.setPower(0);
        Mc2.setPower(0);
        Sg.setPower(0);
    }
    //Trajeto
    public void trajeto() {
        irFrente(2000.0);
        sleep(1000);


        parar();

    }
    public void tel() {
        telemetry.addData("Pose", "%.0f %.0f %.0f", x, y, h);
        telemetry.addData("Erro", "%.0f %.0f %.0f", erX, erY, erH);
        telemetry.addData("Pinpoint", odo.getDeviceStatus());
        if (!aviso.isEmpty()) telemetry.addLine(aviso);
        telemetry.update();
    }
}
