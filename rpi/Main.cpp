#include "Simulation.hpp"


int main()
{
    Simulation simu;
    double Torque[100];
    simu.Simulation::Simulate(0, 0.04, Torque);

    for(double T:Torque)
    {
        std::cout << T << "\n";
    }
    return 0;
}
