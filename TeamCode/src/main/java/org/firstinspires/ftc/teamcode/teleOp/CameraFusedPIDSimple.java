package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "camTracker")
public class CameraFusedPIDSimple extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    // Constantes para cálculo de distância (mantidas para referência, se necessário no futuro)
    public static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 3.75; // Largura real do objeto em polegadas
    public static final double FOCAL_LENGTH = 728; // Distância focal da câmera em pixels

    @Override
    public void runOpMode() {
        // 1. Inicializa a câmera e o pipeline de processamento de imagem
        initOpenCV();

        // 2. Configura o FTC Dashboard para receber telemetria e o stream da câmera
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // 3. Inicia a transmissão da câmera para o Dashboard
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        telemetry.addLine("Aguardando o início do OpMode...");
        telemetry.addLine("Abra o FTC Dashboard para ver a imagem da câmera.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Envia dados sobre o objeto detectado para a telemetria do Driver Station e do Dashboard
            telemetry.addData("Coordenadas do Objeto", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Largura em Pixels", String.format("%.2f", width));
            telemetry.addData("Distância Calculada (pol)", String.format("%.2f", getDistance(width)));
            telemetry.update();
        }

        // Garante que o streaming pare ao finalizar o OpMode
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Define o pipeline que será usado para processar os frames da câmera
        controlHubCam.setPipeline(new BlueBlobDetectionPipeline());

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Inicia o streaming após a câmera ser aberta com sucesso
                controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Lida com erros de abertura da câmera, se necessário
                telemetry.addData("Erro da Câmera", "Não foi possível abrir a câmera. Código: " + errorCode);
                telemetry.update();
            }
        });
    }

    // Pipeline de processamento de imagem para detectar a cor AZUL
    class BlueBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Isola os pixels azuis na imagem
            Mat blueMask = preprocessFrame(input);

            // Encontra os contornos (formas) dos objetos azuis
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Encontra o maior contorno (o maior objeto azul)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Desenha o contorno na imagem que será mostrada no Dashboard
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 255, 0), 2);

                // Calcula a largura e o centro do objeto
                width = calculateWidth(largestContour);
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Desenha um círculo no centro do objeto
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }

            // Retorna a imagem processada (com os desenhos) para ser exibida no Dashboard
            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // Faixa de cor para o AZUL no padrão HSV.
            // Estes valores podem precisar de ajuste dependendo da iluminação do ambiente!
            Scalar lowerBlue = new Scalar(100, 100, 100);
            Scalar upperBlue = new Scalar(140, 255, 255);

            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            // Filtros para reduzir ruído na imagem e melhorar a detecção
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            return largestContour;
        }

        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }
    }

    private static double getDistance(double widthInPixels) {
        if (widthInPixels == 0) {
            return 0; // Evita divisão por zero se nenhum objeto for detectado
        }
        return (OBJECT_WIDTH_IN_REAL_WORLD_UNITS * FOCAL_LENGTH) / widthInPixels;
    }
}