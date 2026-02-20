#include <iostream>
#include <string>
#include <filesystem>
#include <vector>
#include <memory>

// Includes de los dos detectores
#include "charuco_detector/online_detector.hpp"
#include "charuco_detector/offline_detector.hpp"

// Includes de ROS2 (solo para el modo online)
#include "rclcpp/rclcpp.hpp"

std::string get_package_share_path() {
    // Obtener la ruta del paquete basado en la ubicación del ejecutable
    char* path = realpath("/proc/self/exe", nullptr);
    std::string exe_path(path);
    free(path);
    
    // Buscar la parte del path que corresponde al paquete
    size_t pos = exe_path.find("install/charuco_detector/");
    if (pos != std::string::npos) {
        return exe_path.substr(0, pos + 24); // "install/charuco_detector/"
    }
    
    // Fallback
    return "/home/drims/static/drims2_ws/install/charuco_detector/";
}

bool check_for_images_in_pic_folder() {
    std::string base_path = get_package_share_path();
    std::string pic_path = base_path + "share/charuco_detector/config/camera_calibration/pic/";
    
    std::cout << "🔍 Buscando imágenes en: " << pic_path << std::endl;
    
    if (!std::filesystem::exists(pic_path)) {
        std::cout << "📁 La carpeta pic/ no existe" << std::endl;
        return false;
    }
    
    int image_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(pic_path)) {
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        
        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
            image_count++;
        }
    }
    
    std::cout << "📸 Encontradas " << image_count << " imágenes en pic/" << std::endl;
    
    return image_count > 0;
}

void run_offline_mode() {
    std::cout << "\n🔄 MODO OFFLINE - Procesando imágenes de carpeta pic/" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    std::string base_path = get_package_share_path();
    std::string config_path = base_path + "share/charuco_detector/yaml/charuco.yaml";
    std::string calib_path = base_path + "share/charuco_detector/config/camera_calibration/camera_calibration.ini";
    std::string images_path = base_path + "share/charuco_detector/config/camera_calibration/pic/";
    
    // Verificar que existen los archivos necesarios
    if (!std::filesystem::exists(config_path)) {
        std::cerr << "❌ No se encuentra archivo de configuración: " << config_path << std::endl;
        return;
    }
    
    if (!std::filesystem::exists(calib_path)) {
        std::cerr << "❌ No se encuentra calibración de cámara: " << calib_path << std::endl;
        return;
    }
    
    // Crear y ejecutar detector offline
    OfflineChArUcoDetector detector;
    
    if (!detector.loadConfiguration(config_path)) {
        std::cerr << "❌ Error cargando configuración" << std::endl;
        return;
    }
    
    if (!detector.loadCameraCalibration(calib_path)) {
        std::cerr << "❌ Error cargando calibración de cámara" << std::endl;
        return;
    }
    
    detector.processFolder(images_path);
    
    std::cout << "\n✅ Modo OFFLINE completado" << std::endl;
    std::cout << "   Detecciones válidas: " << detector.getValidDetections() 
              << "/" << detector.getTotalImages() << std::endl;
}

void run_online_mode() {
    std::cout << "\n🔄 MODO ONLINE - Streaming en vivo desde cámara" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Inicializar ROS2
    rclcpp::init(0, nullptr);
    
    // Crear detector online
    auto detector = std::make_shared<ChArUcoOnlineDetector>();  // Nombre actualizado
    
    // Configurar y ejecutar
    detector->setupConfigurationFromParameterServer();
    detector->startDetection();
    
    // Mantener ejecución
    rclcpp::spin(detector);
    rclcpp::shutdown();
}

void print_usage() {
    std::cout << "\n📋 USO: charuco_detector [modo]" << std::endl;
    std::cout << "   Modos disponibles:" << std::endl;
    std::cout << "     auto    : Automático (selecciona según imágenes en pic/)" << std::endl;
    std::cout << "     online  : Forzar modo streaming en vivo" << std::endl;
    std::cout << "     offline : Forzar modo procesamiento de carpeta" << std::endl;
    std::cout << "     help    : Mostrar esta ayuda" << std::endl;
    std::cout << std::endl;
    std::cout << "   Ejemplos:" << std::endl;
    std::cout << "     charuco_detector auto    # Modo automático (recomendado)" << std::endl;
    std::cout << "     charuco_detector online   # Forzar streaming" << std::endl;
    std::cout << "     charuco_detector offline  # Forzar procesamiento de imágenes" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    
    std::cout << "\n🎯 ChArUco DETECTOR v2.0 - Modo Inteligente" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    // Determinar modo de ejecución
    std::string mode = "auto";
    
    if (argc > 1) {
        mode = argv[1];
    }
    
    // Mostrar modo seleccionado
    std::cout << "⚙️  Modo seleccionado: " << mode << std::endl;
    
    if (mode == "help" || mode == "--help" || mode == "-h") {
        print_usage();
        return 0;
    }
    
    if (mode == "online") {
        // Forzar modo online
        run_online_mode();
    }
    else if (mode == "offline") {
        // Forzar modo offline
        run_offline_mode();
    }
    else if (mode == "auto") {
        // Modo automático: decidir según contenido de pic/
        std::cout << "🔎 Detectando modo automático..." << std::endl;
        
        bool has_images = check_for_images_in_pic_folder();
        
        if (has_images) {
            std::cout << "✅ Se encontraron imágenes en pic/ - Usando modo OFFLINE" << std::endl;
            run_offline_mode();
        } else {
            std::cout << "✅ No hay imágenes en pic/ - Usando modo ONLINE" << std::endl;
            run_online_mode();
        }
    }
    else {
        std::cerr << "❌ Modo no reconocido: " << mode << std::endl;
        print_usage();
        return 1;
    }
    
    return 0;
}
