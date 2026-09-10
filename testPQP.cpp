#include <iostream>     // Librería para imprimir en pantalla (cout, cerr) y leer de la consola (cin)
#include <fstream>      // Librería para leer y escribir archivos
#include <sstream>      // Librería para manejar flujos de texto
#include <cmath>        // Librería matemática (cos, sin, sqrt, etc.)
#include "PQP.h"        // Librería PQP para detección de colisiones 3D
#include <vector>       // Librería para usar vectores dinámicos (std::vector)
#include <cstring>      // Funciones con cadenas de caracteres tipo C
#include <assimp/Importer.hpp>     // Assimp: cargar modelos 3D
#include <assimp/scene.h>          // Assimp: representación interna de un modelo
#include <assimp/postprocess.h>    // Assimp: procesamiento de modelos al cargarlos

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

// Función para convertir ángulos de Euler (roll, pitch, yaw) a matriz de rotación 3x3
void rpyToMatrix(double roll, double pitch, double yaw, PQP_REAL R[3][3]) {
    // Calcular cosenos y senos de cada ángulo
    double cr = cos(roll);   // coseno de roll
    double sr = sin(roll);   // seno de roll
    double cp = cos(pitch);  // coseno de pitch
    double sp = sin(pitch);  // seno de pitch
    double cy = cos(yaw);    // coseno de yaw
    double sy = sin(yaw);    // seno de yaw

    // Llenar la matriz de rotación según la fórmula de Euler ZYX
    R[0][0] = cp * cy;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

// ============================================================================
// Función para cargar un modelo STL usando Assimp y convertirlo a PQP_Model
// ============================================================================
void loadModel(const std::string &filename, PQP_Model &model) {
    Assimp::Importer importer;  // Crear un "importador" de modelos
    // Leer el archivo STL y convertirlo a triángulos, invertir UVs si es necesario
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_FlipUVs);

    // Verificar si se cargó correctamente
    if (!scene) {
        std::cerr << "Error: unable to load model " << filename 
                  << " - " << importer.GetErrorString() << std::endl;
        exit(1); // Terminar el programa si no se puede cargar el modelo
    }

    // Iterar sobre todas las mallas del modelo (un modelo puede tener varias partes)
    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[i];

        // Iterar sobre todas las caras de la malla
        for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
            aiFace face = mesh->mFaces[j];
            
            // PQP requiere que cada cara sea un triángulo
            if (face.mNumIndices == 3) {
                // Obtener los índices de los vértices del triángulo
                unsigned int idx0 = face.mIndices[0];
                unsigned int idx1 = face.mIndices[1];
                unsigned int idx2 = face.mIndices[2];

                // Obtener las coordenadas de cada vértice
                aiVector3D vertex0 = mesh->mVertices[idx0];
                aiVector3D vertex1 = mesh->mVertices[idx1];
                aiVector3D vertex2 = mesh->mVertices[idx2];

                // Convertir a formato PQP_REAL (tipo que usa PQP)
                PQP_REAL pqpV0[3] = {vertex0.x, vertex0.y, vertex0.z};
                PQP_REAL pqpV1[3] = {vertex1.x, vertex1.y, vertex1.z};
                PQP_REAL pqpV2[3] = {vertex2.x, vertex2.y, vertex2.z};

                // Agregar el triángulo al modelo PQP
                model.AddTri(pqpV0, pqpV1, pqpV2, j);
            }
        }
    }

    // Finalizar la construcción del modelo PQP
    model.EndModel();
}

// ============================================================================
// Función que verifica si dos modelos 3D están en colisión
// ============================================================================
bool collisionChecker(PQP_Model &model1, const std::vector<double>& state1,
                      PQP_Model &model2, const std::vector<double>& state2) {
    // Validar que cada estado tenga exactamente 6 elementos: [x, y, z, roll, pitch, yaw]
    if (state1.size() != 6 || state2.size() != 6) {
        std::cerr << "Error: cada estado debe tener 6 elementos (x, y, z, roll, pitch, yaw)" << std::endl;
        return false;
    }

    // Extraer la posición y orientación del primer modelo
    double x1 = state1[0], y1 = state1[1], z1 = state1[2];
    double roll1 = state1[3], pitch1 = state1[4], yaw1 = state1[5];

    // Extraer la posición y orientación del segundo modelo
    double x2 = state2[0], y2 = state2[1], z2 = state2[2];
    double roll2 = state2[3], pitch2 = state2[4], yaw2 = state2[5];

    // Matrices de rotación y vectores de posición para PQP
    PQP_REAL R1[3][3], T1[3];
    PQP_REAL R2[3][3], T2[3];

    // Convertir ángulos a matrices de rotación
    rpyToMatrix(roll1, pitch1, yaw1, R1);
    rpyToMatrix(roll2, pitch2, yaw2, R2);

    // Asignar las posiciones (traducciones)
    T1[0] = x1; T1[1] = y1; T1[2] = z1;
    T2[0] = x2; T2[1] = y2; T2[2] = z2;

    // Variable para almacenar el resultado de colisión
    PQP_CollideResult result;

    // Verificar colisión entre los dos modelos
    PQP_Collide(&result, R1, T1, &model1, R2, T2, &model2);

    // Retornar true si están en colisión, false si no
    return result.Colliding();
}

// ============================================================================
// FUNCIÓN PRINCIPAL
// ============================================================================
int main() {
    // Crear dos modelos PQP vacíos
    PQP_Model model1;
    PQP_Model model2;

    // Cargar modelos 3D desde archivos STL
    loadModel("models/carModel.stl", model1);
    loadModel("models/Entorno1-TesisRRT.stl", model2);

    // Definir estados de ejemplo: [x, y, z, roll, pitch, yaw]
    std::vector<double> state1 = {-2000, -2000, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> state2 = {-2500, -2500, 0.0, 0.0, 0.0, 0.0};

    // Verificar si los modelos están en colisión
    bool collision = collisionChecker(model1, state1, model2, state2);

    // Imprimir el resultado
    if (collision) {
        std::cout << "¡Los modelos están en colisión!" << std::endl;
    } else {
        std::cout << "Los modelos no están en colisión." << std::endl;
    }

    return 0; // Termina el programa
}
