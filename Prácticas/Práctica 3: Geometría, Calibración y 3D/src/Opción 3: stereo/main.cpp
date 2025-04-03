#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

int main() {
    // Cargar las imágenes rectificadas en escala de grises
    cv::Mat left = cv::imread("pinkballs_1.jpeg", cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread("pinkballs_2.jpeg", cv::IMREAD_GRAYSCALE);

    if(left.empty() || right.empty()) {
        std::cerr << "Error: No se pudieron cargar las imágenes estéreo rectificadas" << std::endl;
        return -1;
    }

    // Parámetros para StereoBM
    int numDisparities = 64;   // Debe ser divisible por 16
    int blockSize = 15;        // Tamaño del bloque (debe ser impar)

    // Crear el objeto StereoBM
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(numDisparities, blockSize);

    // Configurar parámetros adicionales (opcional)
    stereo->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
    stereo->setPreFilterSize(9);
    stereo->setPreFilterCap(31);
    stereo->setTextureThreshold(10);
    stereo->setUniquenessRatio(15);
    stereo->setSpeckleWindowSize(100);
    stereo->setSpeckleRange(32);
    stereo->setDisp12MaxDiff(1);

    // Calcular el mapa de disparidad
    cv::Mat disparity, disparityNormalized;
    stereo->compute(left, right, disparity);

    // Normalizar el mapa de disparidad para visualización
    cv::normalize(disparity, disparityNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Mostrar resultados
    cv::namedWindow("Imagen izquierda rectificada", cv::WINDOW_NORMAL);
    cv::imshow("Imagen izquierda rectificada", left);
    
    cv::namedWindow("Imagen derecha rectificada", cv::WINDOW_NORMAL);
    cv::imshow("Imagen derecha rectificada", right);
    
    cv::namedWindow("Mapa de disparidad", cv::WINDOW_NORMAL);
    cv::imshow("Mapa de disparidad", disparityNormalized);

    // Guardar el mapa de disparidad (opcional)
    // cv::imwrite("disparity_map.jpg", disparityNormalized);

    cv::waitKey(0);
    return 0;
}
