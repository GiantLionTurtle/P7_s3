#include "Simulation_V2.hpp"


int main()
{
    Simulation_V2 simu(0.04);
    std::vector<double> Sequence = simu.RunSimulation(0, 1.4, 100, 0, 0);

    for(auto s : Sequence)
    {
      std::cout << s << "\n";
    }
}
