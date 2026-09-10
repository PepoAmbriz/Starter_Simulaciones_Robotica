#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <cmath>
#include <cstdlib>
#include <random>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/StateSpace.h>

#include <iostream>
#include <vector>
#include <string>


using namespace std;
using Vec = std::vector<double>;  // Cambié 'vector' a 'Vec' para evitar ambigüedad

const int width = 1000;
const int height = 1000;


// Variables globales
Vec robot = {0, 0, 0};
const double L = 30.0;
bool iniciar = false;

// Colores y radio de giro
const float green[] = {0.0f, 1.0f, 0.0f, 1.0f};
const float red[] = {1.0f, 0.0f, 0.0f, 1.0f};
const float blue[] = {0.0f, 0.0f, 1.0f, 1.0f};

double phi = M_PI / 5;
double radio_giro = L / tan(phi);

// Generador aleatorio
std::random_device rd;
std::mt19937 gen(rd());

// Generar estado aleatorio para el goal
Vec generateRandomGoal() {
    std::uniform_int_distribution<> distribX(-100, 100);
    std::uniform_int_distribution<> distribY(-100, 100);
    std::uniform_real_distribution<> distribTheta(0, 2 * M_PI);
    return {static_cast<double>(distribX(gen)), static_cast<double>(distribY(gen)), distribTheta(gen)};
}

// Inicializar GLFW y OpenGL
void myInit() {
    if (!glfwInit()) {
        throw std::runtime_error("No se pudo inicializar GLFW");
    }

    GLFWwindow* window = glfwCreateWindow(width, height, "STEER CAR", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("No se pudo crear la ventana");
    }

    glfwMakeContextCurrent(window);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double scale_factor = 0.3;
    glOrtho(-width / 2 * scale_factor, width / 2 * scale_factor,
            -height / 2 * scale_factor, height / 2 * scale_factor, -1, 1);
    glMatrixMode(GL_MODELVIEW);
}

// Manejar eventos del teclado
void keyCallback(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int /*mods*/) {
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        iniciar = !iniciar;
    }
}


// Dibujar línea entre dos vectores de 3 elementos (x, y, yaw), como rectangulos
void drawLine(const Vec& point1, const Vec& point2, const float* color, float thickness) {
    double dx = point2[0] - point1[0];
    double dy = point2[1] - point1[1];
    double L = sqrt(dx * dx + dy * dy);
    double perpX = dy / L;
    double perpY = -dx / L;
    double halfThickness = thickness / 2;
    glColor4fv(color);
    glBegin(GL_TRIANGLES);
    glVertex2f(point1[0] + perpX * halfThickness, point1[1] + perpY * halfThickness);
    glVertex2f(point1[0] - perpX * halfThickness, point1[1] - perpY * halfThickness);
    glVertex2f(point2[0] + perpX * halfThickness, point2[1] + perpY * halfThickness);
    glVertex2f(point2[0] + perpX * halfThickness, point2[1] + perpY * halfThickness);
    glVertex2f(point1[0] - perpX * halfThickness, point1[1] - perpY * halfThickness);
    glVertex2f(point2[0] - perpX * halfThickness, point2[1] - perpY * halfThickness);
    glEnd();
}

// Dibujar flecha orientada
void drawArrow(const Vec& point, double L, float thickness, const float* color) {
    double x1 = point[0] + L * cos(point[2]);
    double y1 = point[1] + L * sin(point[2]);
    drawLine(point, {x1, y1, point[2]}, color, thickness);
    glBegin(GL_TRIANGLES);
    glVertex2f(x1, y1);
    glVertex2f(x1 - 10 * cos(point[2] + M_PI / 6), y1 - 10 * sin(point[2] + M_PI / 6));
    glVertex2f(x1 - 10 * cos(point[2] - M_PI / 6), y1 - 10 * sin(point[2] - M_PI / 6));
    glEnd();
}

// Dibujar el camino que es un vector de vectores
void drawPath(const std::vector<Vec>& path, const float* color = green) {
    int tamano = path.size();
    for (int i = 0; i < tamano - 1; ++i) {
        drawLine(path[i], path[i + 1], color, 1.0f);
    }
}

auto Path(const Vec& x0, const Vec& xg, double radio_giro , bool restringido = false) {
    // Crear un espacio de estados Reeds-Shepp con un radio de giro 
    ompl::base::ReedsSheppStateSpace space(radio_giro);

    // Asignar memoria para los estados
    ompl::base::StateSpacePtr ss = std::make_shared<ompl::base::ReedsSheppStateSpace>(radio_giro); 
    ompl::base::State *state1 = ss->allocState();
    ompl::base::State *state2 = ss->allocState();
    ompl::base::State *interpolatedState = ss->allocState();  // Para guardar el estado interpolado

    // Definir las posiciones y orientaciones de los dos estados
    state1->as<ompl::base::SE2StateSpace::StateType>()->setXY(x0[0], x0[1]);
    state1->as<ompl::base::SE2StateSpace::StateType>()->setYaw(x0[2]);  

    state2->as<ompl::base::SE2StateSpace::StateType>()->setXY(xg[0], xg[1]);
    state2->as<ompl::base::SE2StateSpace::StateType>()->setYaw(xg[2]);  

    // Obtener el camino Reeds-Shepp más corto entre los dos estados
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath path = space.reedsShepp(state1, state2);
    // if (restringido) 
    // ompl::base::ReedsSheppStateSpace::ReedsSheppPath path = space.reedsSheppEmil(state1, state2);
    


    // Imprimir los tipos de segmentos del camino y si es hacia adelante o en reversa
    for (int i = 0; i < 5; ++i) {
        if (path.length_[i] == 0)  // Si el segmento no existe, no lo imprimimos
            continue;

        string direction = (path.length_[i] > 0) ? "+" : "-";
        switch (path.type_[i]) {
            case ompl::base::ReedsSheppStateSpace::RS_LEFT:
                cout << "L" << direction << endl;
                break;
            case ompl::base::ReedsSheppStateSpace::RS_RIGHT:
                cout << "R" << direction << endl;
                break;
            case ompl::base::ReedsSheppStateSpace::RS_STRAIGHT:
                cout << "S" << direction << endl;
                break;
            case ompl::base::ReedsSheppStateSpace::RS_NOP:
                break;
        }
    }

    // Obtener e imprimir puntos a lo largo de la trayectoria
    int numPoints = 100;  // Número de puntos que queremos obtener
    std::vector<Vec> Puntos(numPoints + 1, Vec(3));  // Vector dinámico para almacenar los puntos (x, y, yaw)

    for (int i = 0; i <= numPoints; ++i) {
        double t = (double)i / numPoints;  // Proporción de interpolación [0, 1]
        space.interpolate(state1, state2, t, interpolatedState);

        // Obtener coordenadas X, Y y orientación (Yaw) del estado interpolado
        auto *se2state = interpolatedState->as<ompl::base::SE2StateSpace::StateType>();
        double x = se2state->getX();
        double y = se2state->getY();
        double yaw = se2state->getYaw();

        // Guardar en Vec
        Puntos[i] = Vec{x, y, yaw};  // Usar el constructor de Vec
    }

    // Liberar memoria de los estados
    ss->freeState(state1);
    ss->freeState(state2);
    ss->freeState(interpolatedState);

    return Puntos;
}


// Función principal
int main() {
    myInit();
    GLFWwindow* window = glfwGetCurrentContext();
    glfwSetKeyCallback(window, keyCallback);
    
    Vec goal = {30,0,0};
    std::vector<Vec> Points;  // Declarar Points aquí
    std::vector<Vec> PointsRestringidos;  

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);

        if (iniciar) {
            goal = generateRandomGoal();
            // Obtenemos la solucion con RS
            Points = Path(robot, goal,30);  // Aquí se asigna el valor a Points
            // Obtenemos la solucion con RS restringido
            //PointsRestringidos = Path(robot, goal,30, true);  // Aquí se asigna el valor a Points
            iniciar = false;
            //Calcular toda la dsiatncia del camino
            double totalLength = 0.0;
            //double totalLength2 = 0.0; 
            for (size_t i = 0; i < Points.size()-1; ++i) {
                double dx = Points[i+1][0] - Points[i][0];
                //double dx_2 = PointsRestringidos[i+1][0] - PointsRestringidos[i][0];

                double dy = Points[i+1][1] - Points[i][1];
               // double dy_2 = PointsRestringidos[i+1][1] - PointsRestringidos[i][1];

                totalLength += sqrt(dx * dx + dy * dy);
               // totalLength2 += sqrt(dx_2 * dx_2 + dy_2 * dy_2);

            }
            cout << "Longitud Óptima: " << totalLength << endl;
           // cout << "Longitud Restringida: " << totalLength2 << endl;
        }

        drawArrow(robot, L, 1.0f, blue);  // Dibujar el robot
        drawArrow(goal, L, 1.0f, red);    // Dibujar el goal
        drawPath(Points);                 // Dibujar el camino
        drawPath(PointsRestringidos, blue);     // Dibujar el camino restringido
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
