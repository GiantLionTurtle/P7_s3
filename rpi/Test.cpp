#include "Simulation_V3.hpp"
#include <iostream>


int main()
{

    std::vector<double> Seq = runSimulation(0, 1.373468, 100, 0, 0);
    std::cout << "Premiere Simu:\n";
    for(auto s: Seq)
    {
        std::cout << s << "\n";
    }

    Seq = runSimulation(0, 0.1, 10, 0, 0);
    std::cout << "Deuxieme Simu:\n";
    for(auto s: Seq)
    {
        std::cout << s << "\n";
    }
    return 0;
}