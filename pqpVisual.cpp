#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include "PQP.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glu.h>

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

// Convertir roll, pitch, yaw a matriz de rotación 3x3
void rpyToMatrix(double roll, double pitch, double yaw, PQP_REAL R[3][3]) {
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    R[0][0] = cp * cy;    R[0][1] = cy * sp * sr - sy * cr; R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;    R[1][1] = sy * sp * sr + cy * cr; R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;        R[2][1] = cp * sr;               R[2][2] = cp * cr;
}

// Cargar modelo STL con Assimp y agregar a PQP_Model
void loadModel(const std::string &filename, PQP_Model &model, std::vector<std::vector<PQP_REAL>> &triangles) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate | aiProcess_FlipUVs);
    if (!scene) {
        std::cerr << "Error cargando modelo " << filename << ": " << importer.GetErrorString() << std::endl;
        exit(1);
    }

    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[i];
        for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
            aiFace face = mesh->mFaces[j];
            if (face.mNumIndices == 3) {
                PQP_REAL p0[3], p1[3], p2[3];
                aiVector3D v0 = mesh->mVertices[face.mIndices[0]];
                aiVector3D v1 = mesh->mVertices[face.mIndices[1]];
                aiVector3D v2 = mesh->mVertices[face.mIndices[2]];

                p0[0] = v0.x; p0[1] = v0.y; p0[2] = v0.z;
                p1[0] = v1.x; p1[1] = v1.y; p1[2] = v1.z;
                p2[0] = v2.x; p2[1] = v2.y; p2[2] = v2.z;

                model.AddTri(p0, p1, p2, j);

                // Guardar triángulo para dibujar en OpenGL
                triangles.push_back({p0[0], p0[1], p0[2],
                                     p1[0], p1[1], p1[2],
                                     p2[0], p2[1], p2[2]});
            }
        }
    }

    model.EndModel();
}

// Verificar colisión entre dos modelos
bool collisionChecker(PQP_Model &model1, const std::vector<double>& state1,
                      PQP_Model &model2, const std::vector<double>& state2) {
    if (state1.size() != 6 || state2.size() != 6) return false;

    PQP_REAL R1[3][3], T1[3], R2[3][3], T2[3];
    rpyToMatrix(state1[3], state1[4], state1[5], R1);
    rpyToMatrix(state2[3], state2[4], state2[5], R2);
    T1[0]=state1[0]; T1[1]=state1[1]; T1[2]=state1[2];
    T2[0]=state2[0]; T2[1]=state2[1]; T2[2]=state2[2];

    PQP_CollideResult result;
    PQP_Collide(&result, R1, T1, &model1, R2, T2, &model2);
    return result.Colliding();
}

// ============================================================================
// CAMARA
// ============================================================================

// Estado de la vista orbital.
double cameraYaw = 0.0;
double cameraPitch = M_PI / 2.0;
double cameraDistance = 8000.0;
double lastMouseX = 0.0;
double lastMouseY = 0.0;
bool mouseDragging = false;

// Arrastrar con el mouse rota la vista alrededor del origen.
void onMouseButton(GLFWwindow *window, int button, int action, int mods) {
    (void)window;
    (void)mods;

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            mouseDragging = true;
            glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
        } else if (action == GLFW_RELEASE) {
            mouseDragging = false;
        }
    }
}

// Actualizar yaw y pitch cuando se mueve el mouse con click izquierdo presionado.
void onCursorPos(GLFWwindow *window, double xpos, double ypos) {
    (void)window;

    if (!mouseDragging) return;

    double dx = xpos - lastMouseX;
    double dy = ypos - lastMouseY;
    lastMouseX = xpos;
    lastMouseY = ypos;

    cameraYaw += dx * 0.005;
    cameraPitch -= dy * 0.005;
    cameraPitch = std::clamp(cameraPitch, -1.56, 1.56);
}

// ============================================================================
// FUNCIONES OPENGL
// ============================================================================

// Dibujar un modelo usando los triángulos
void drawModel(const std::vector<std::vector<PQP_REAL>>& triangles, const std::vector<double>& state) {
    glPushMatrix(); // Guardar la transformación actual
    // Aplicar traslación
    glTranslated(state[0], state[1], state[2]);
    // Aplicar rotación ZYX
    glRotated(state[5]*180.0/M_PI, 0,0,1); // yaw
    glRotated(state[4]*180.0/M_PI, 0,1,0); // pitch
    glRotated(state[3]*180.0/M_PI, 1,0,0); // roll

    // Dibujar triángulos
    glBegin(GL_TRIANGLES);
    for (const auto& tri : triangles) {
        glVertex3d(tri[0], tri[1], tri[2]);
        glVertex3d(tri[3], tri[4], tri[5]);
        glVertex3d(tri[6], tri[7], tri[8]);
    }
    glEnd();

    glPopMatrix(); // Restaurar transformación
}

// ============================================================================
// MAIN
// ============================================================================
int main() {
    // Crear modelos PQP y contenedores de triángulos
    PQP_Model model1, model2;
    std::vector<std::vector<PQP_REAL>> tris1, tris2;

    // Cargar modelos
    loadModel("models/carModel.stl", model1, tris1);
    loadModel("models/Entorno1-TesisRRT.stl", model2, tris2);

    // Estados de ejemplo
    std::vector<double> state1 = {-2000, -2000, 0, 0, 0, 0};
    std::vector<double> state2 = {-2500, -2500, 0, 0, 0, 0};

    // Inicializar GLFW
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(1000, 1000, "Visualizador 3D", NULL, NULL);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, onMouseButton);
    glfwSetCursorPosCallback(window, onCursorPos);

    // Configuración básica de OpenGL
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, 1200.0/1200.0, 1, 10000);
    glMatrixMode(GL_MODELVIEW);

    // Loop principal
    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        // Vista orbital alrededor del origen con rotación por mouse.
        double eyeX = cameraDistance * std::cos(cameraPitch) * std::cos(cameraYaw);
        double eyeY = cameraDistance * std::cos(cameraPitch) * std::sin(cameraYaw);
        double eyeZ = cameraDistance * std::sin(cameraPitch);
        gluLookAt(eyeX, eyeY, eyeZ, 0,0,0, 0,0,1);


        //actualizar state 1 con flechas del teclado 
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) state1[1] += 10;
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) state1[1] -= 10;
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) state1[0] -= 10;
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) state1[0] += 10;


        // Dibujar modelos, si colisionan poner el 1 en rojo
        if (collisionChecker(model1, state1, model2, state2)) {
            glColor3f(1,0,0); 
            drawModel(tris1, state1); // rojo
        } else {
            glColor3f(0,0,1); 
            drawModel(tris1, state1); // azul
        }
        glColor3f(0,1,0); 
        drawModel(tris2, state2); // verde






        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
