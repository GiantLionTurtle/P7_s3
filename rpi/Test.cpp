#include "Simulation_V2.hpp"


int main()
{
    Simulation_V2 sim(0.04);

    std::vector<double> Seq = sim.RunSimulation(0, 1.373468, 100, 0, 0);

    for(auto s: Seq)
    {
        std::cout << s << "\n";
    }

    Seq = sim.RunSimulation(0, 0.1, 10, 0, 0);
    std::cout << "Deuxieme Simu:\n";
    for(auto s: Seq)
    {
        std::cout << s << "\n";
    }
    return 0;
}