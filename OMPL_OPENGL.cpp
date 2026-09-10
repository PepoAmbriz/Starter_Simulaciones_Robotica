// RRT* realtime con path + aristas (usar PlannerData, seguro)
// Compilar con tus includes/links habituales (igual que tu task)

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/PlannerData.h>

#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include <iostream>
#include <vector>
#include <utility>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cmath>
#include <random>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Point2 = std::pair<double,double>;
using Polyline = std::vector<Point2>;
using Polylines = std::vector<Polyline>;

// ------------------ Snapshot protegido ------------------
Polylines edgesSnapshot;             // aristas muestreadas (para render)
std::vector<Point2> pathSnapshot;    // path muestreado (para render)
std::mutex mutexSnapshot;
std::atomic<bool> planningDone(false);
std::atomic<bool> running(true);
std::atomic<bool> restartRequested(false);

// SPACE detiene la mejor ruta actual; S pide reiniciar con una meta nueva.
void onKey(GLFWwindow *window, int key, int scancode, int action, int mods) {
    (void)window;
    (void)scancode;
    (void)mods;

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        running.store(false);
        std::cout << "Planificacion detenida con SPACE. Se mantiene la mejor ruta encontrada.\n";
    } else if (key == GLFW_KEY_S && action == GLFW_PRESS) {
        restartRequested.store(true);
        running.store(false);
        std::cout << "Reinicio solicitado con S. Se generara una meta aleatoria.\n";
    }
}

// ------------------ Inicio / Meta ------------------
double startX = 0.0, startY = 0.0, startYaw = 0.0;
double goalX  = 3.0, goalY  = 4.0, goalYaw  = M_PI/4;

// ------------------ Visual ------------------
double lineWidthWorld = 0.03;
unsigned int samplesPerEdge = 12;
double arrowLength = 0.4;
double arrowWidth  = 0.1;

// ------------------ Validación de estado ------------------
bool isStateValid(const ob::State *state) {
    const auto *rs = state->as<ob::ReedsSheppStateSpace::StateType>();
    double x = rs->getX(), y = rs->getY();
    return !(x < -5.0 || x > 5.0 || y < -5.0 || y > 5.0);
}

void clearSnapshots() {
    std::lock_guard<std::mutex> lock(mutexSnapshot);
    edgesSnapshot.clear();
    pathSnapshot.clear();
}

// Genera una meta nueva dentro del espacio permitido antes de arrancar otra corrida.
void randomizeGoal() {
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> xyDist(-5.0, 5.0);
    std::uniform_real_distribution<double> yawDist(-M_PI, M_PI);

    goalX = xyDist(rng);
    goalY = xyDist(rng);
    goalYaw = yawDist(rng);

    std::cout << "Nueva meta aleatoria: x=" << goalX
              << ", y=" << goalY
              << ", yaw=" << goalYaw << "\n";
}

// ------------------ Dibujo polylines (genérico) ------------------
void drawThickPolyline(const Polyline &poly, double halfWidth) {
    if (poly.size()<2) return;
    for(size_t i=0;i+1<poly.size();++i){
        float x1 = (float)poly[i].first, y1 = (float)poly[i].second;
        float x2 = (float)poly[i+1].first, y2 = (float)poly[i+1].second;
        float vx = x2-x1, vy=y2-y1;
        float len = std::sqrt(vx*vx + vy*vy);
        if (len<=1e-9f) continue;
        float nx=-vy/len, ny=vx/len;
        float ox=nx*(float)halfWidth, oy=ny*(float)halfWidth;
        float v1x=x1+ox,v1y=y1+oy;
        float v2x=x1-ox,v2y=y1-oy;
        float v3x=x2+ox,v3y=y2+oy;
        float v4x=x2-ox,v4y=y2-oy;
        glBegin(GL_TRIANGLES);
            glVertex2f(v1x,v1y);
            glVertex2f(v2x,v2y);
            glVertex2f(v3x,v3y);
            glVertex2f(v3x,v3y);
            glVertex2f(v2x,v2y);
            glVertex2f(v4x,v4y);
        glEnd();
    }
}

// ------------------ Flecha ------------------
void drawArrow(double x, double y, double yaw, double length, double width){
    double cosY = std::cos(yaw), sinY = std::sin(yaw);
    double tipX = x + length*cosY;
    double tipY = y + length*sinY;
    double leftX  = x - width*sinY;
    double leftY  = y + width*cosY;
    double rightX = x + width*sinY;
    double rightY = y - width*cosY;

    glBegin(GL_TRIANGLES);
        glVertex2f((float)tipX, (float)tipY);
        glVertex2f((float)leftX, (float)leftY);
        glVertex2f((float)rightX, (float)rightY);
    glEnd();
}

// ------------------ Hilo de planificación ------------------
void planWorker(unsigned int maxIterations, unsigned int snapshotMs, double sliceSeconds){
    try {
        // espacio Reeds-Shepp
        auto space = std::make_shared<ob::ReedsSheppStateSpace>(1.0);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0,-5.0); bounds.setHigh(0,5.0);
        bounds.setLow(1,-5.0); bounds.setHigh(1,5.0);
        space->setBounds(bounds);

        og::SimpleSetup ss(space);
        ss.setStateValidityChecker([](const ob::State *s){ return isStateValid(s); });

        // start
        ob::ScopedState<> start(space);
        start->as<ob::ReedsSheppStateSpace::StateType>()->setX(startX);
        start->as<ob::ReedsSheppStateSpace::StateType>()->setY(startY);
        start->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(startYaw);

        // goal
        ob::ScopedState<> goal(space);
        goal->as<ob::ReedsSheppStateSpace::StateType>()->setX(goalX);
        goal->as<ob::ReedsSheppStateSpace::StateType>()->setY(goalY);
        goal->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(goalYaw);

        ss.setStartAndGoalStates(start, goal);

        // planner RRT*
        auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
        ss.setPlanner(planner);
        ss.setup();

        // temporal storage
        Polylines localEdges;
        std::vector<Point2> localPath;

        for(unsigned int iterations=0; iterations<maxIterations && running.load(); ++iterations){
            ss.solve(sliceSeconds); // slice

            // --- Obtener PlannerData (safe) y construir aristas muestreadas ---
            try {
                ob::PlannerData pd(ss.getSpaceInformation());
                ss.getPlannerData(pd);              // llena pd con los vértices/aristas actuales
                pd.decoupleFromPlanner();           // hace deep-copy de los estados -> seguro usar fuera del planner

                localEdges.clear();
                // recorremos vértices y sus aristas salientes
                unsigned int nVertices = pd.numVertices();
                for (unsigned int v = 0; v < nVertices; ++v) {
                    // estado del vértice v
                    const ob::PlannerDataVertex &pv = pd.getVertex(v);
                    const ob::State *sv = pv.getState();
                    if (!sv) continue;

                    // aristas salientes desde v
                    std::vector<unsigned int> outEdges;
                    pd.getEdges(v, outEdges);

                    for (unsigned int toIdx : outEdges) {
                        if (toIdx >= pd.numVertices()) continue;
                        const ob::PlannerDataVertex &pv2 = pd.getVertex(toIdx);
                        const ob::State *su = pv2.getState();
                        if (!su) continue;

                        // muestrear entre sv y su
                        Polyline poly; poly.reserve(samplesPerEdge);
                        ob::State* tmp = ss.getSpaceInformation()->getStateSpace()->allocState();
                        for (unsigned int s = 0; s < samplesPerEdge; ++s) {
                            double t = double(s)/(samplesPerEdge-1);
                            ss.getSpaceInformation()->getStateSpace()->interpolate(sv, su, t, tmp);
                            const auto *rs = tmp->as<ob::ReedsSheppStateSpace::StateType>();
                            poly.emplace_back(rs->getX(), rs->getY());
                        }
                        ss.getSpaceInformation()->getStateSpace()->freeState(tmp);
                        localEdges.push_back(std::move(poly));
                    }
                }

                // actualizar snapshot de aristas (bajo lock)
                {
                    std::lock_guard<std::mutex> lock(mutexSnapshot);
                    edgesSnapshot = std::move(localEdges);
                }
            } catch(...) {
                // no interrumpir planificación si pd falla; seguir con path snapshot
            }

            // --- snapshot del camino parcial (PathGeometric) ---
            og::PathGeometric path = ss.getSolutionPath();
            if(path.getStateCount()>0){
                path.interpolate();
                localPath.clear();
                localPath.reserve(path.getStateCount());
                for(size_t i=0;i<path.getStateCount();++i){
                    const auto *rs = path.getState(i)->as<ob::ReedsSheppStateSpace::StateType>();
                    localPath.emplace_back(rs->getX(), rs->getY());
                }
                {
                    std::lock_guard<std::mutex> lock(mutexSnapshot);
                    pathSnapshot = std::move(localPath);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(snapshotMs));
        }
    } catch(const std::exception &e) {
        std::cerr << "Exception en planWorker: " << e.what() << std::endl;
    } catch(...) {
        std::cerr << "Excepción no capturada en planWorker\n";
    }

    planningDone.store(true);
}

// ------------------ Dibujo escena ------------------
void drawScene(){
    glClear(GL_COLOR_BUFFER_BIT);

    Polylines edgesLocal;
    std::vector<Point2> pathLocal;
    {
        std::lock_guard<std::mutex> lock(mutexSnapshot);
        edgesLocal = edgesSnapshot;
        pathLocal = pathSnapshot;
    }

    double halfW = lineWidthWorld*0.5;
    double edgeHalfWidth = halfW * 0.5; // aristas más delgadas
    double pathHalfWidth = halfW * 1.5; // path más grueso

    // dibujar aristas negras (delgadas)
    glColor3f(0.0f,0.0f,0.0f);
    for(const auto &poly: edgesLocal) drawThickPolyline(poly, edgeHalfWidth);

    // dibujar camino rojo (grueso)
    if(!pathLocal.empty()){
        glColor3f(1.0f,0.0f,0.0f);
        drawThickPolyline(pathLocal, pathHalfWidth);
    }

    // flechas inicio/meta
    glColor3f(0.0f,0.0f,1.0f);
    drawArrow(startX, startY, startYaw, arrowLength, arrowWidth);
    glColor3f(0.0f,1.0f,0.0f);
    drawArrow(goalX, goalY, goalYaw, arrowLength, arrowWidth);

    glFlush();
}

// ------------------ Main ------------------
int main(){
    const unsigned int maxIterations = 1000;
    const unsigned int snapshotMs = 25; // ms entre snapshots
    const double sliceSeconds = 0.01;   // 10 ms por slice

    // crear ventana antes de arrancar el hilo (más estable en WSL)
    if(!glfwInit()){
        std::cerr<<"Error glfwInit\n";
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(900,900,"RRT* realtime (edges+path)",nullptr,nullptr);
    if(!window){
        std::cerr<<"Error crear ventana\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, onKey);

    // projection
    glClearColor(0.85f,0.85f,0.85f,1.0f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-6,6,-6,6,-1,1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // arrancar hilo planner
    std::thread plannerThread(planWorker, maxIterations, snapshotMs, sliceSeconds);

    // loop render
    while(!glfwWindowShouldClose(window)){
        drawScene();
        glfwSwapBuffers(window);
        glfwPollEvents();

        if (restartRequested.exchange(false)) {
            // Cerrar el ciclo actual, limpiar lo visible y levantar otra planificación.
            running.store(false);
            if (plannerThread.joinable()) {
                plannerThread.join();
            }

            clearSnapshots();
            planningDone.store(false);
            randomizeGoal();
            running.store(true);
            plannerThread = std::thread(planWorker, maxIterations, snapshotMs, sliceSeconds);
        }

        if (glfwWindowShouldClose(window)) running.store(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    running.store(false);
    if(plannerThread.joinable()) plannerThread.join();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
