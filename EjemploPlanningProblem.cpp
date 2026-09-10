#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// ------------------ Función de validación de estado ------------------
bool isStateValid(const ob::State *state)
{
    const auto *reedssheppState = state->as<ob::ReedsSheppStateSpace::StateType>();
    double x = reedssheppState->getX();
    double y = reedssheppState->getY();

    // Validación básica de límites
    if (x < -5.0 || x > 5.0 || y < -5.0 || y > 5.0)
        return false;
    return true;
}

// ------------------ Planificación ------------------
void planWithReedsShepp()
{
    // Crear espacio Reeds-Shepp
    auto space = std::make_shared<ob::ReedsSheppStateSpace>(1.0); // radio de giro

    // Limites del espacio
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -5);
    bounds.setHigh(0, 5);
    bounds.setLow(1, -5);
    bounds.setHigh(1, 5);
    space->setBounds(bounds);

    // Configuración simple
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    // Estado inicial
    ob::ScopedState<> start(space);
    start->as<ob::ReedsSheppStateSpace::StateType>()->setX(0);
    start->as<ob::ReedsSheppStateSpace::StateType>()->setY(0);
    start->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(0);

    // Estado meta aleatorio
    ob::ScopedState<> goal(space);
    goal.random();
    printf("Goal: (%f, %f, %f)\n",
           goal->as<ob::ReedsSheppStateSpace::StateType>()->getX(),
           goal->as<ob::ReedsSheppStateSpace::StateType>()->getY(),
           goal->as<ob::ReedsSheppStateSpace::StateType>()->getYaw());

    ss.setStartAndGoalStates(start, goal);

    // Planificador RRT*
    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);
    ss.setup();

    // ------------------ Expandir el árbol por "slices" de tiempo ------------------
    unsigned int maxIterations = 1000;
    double sliceSeconds = 0.001; // 1 ms por slice (ajusta según necesites)

    for (unsigned int i = 0; i < maxIterations; ++i)
    {
        // Llamar a solve con un pequeño tiempo permite expandir el planner sin bloquear
        ss.solve(sliceSeconds);
    }

    // Obtener la solución si existe
    if (ss.haveSolutionPath())
    {
        ss.simplifySolution();
        og::PathGeometric path = ss.getSolutionPath();
        path.interpolate();
        path.printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No se encontró solución\n";
    }
}

// ------------------ Main ------------------
int main()
{
    planWithReedsShepp();
    return 0;
}

