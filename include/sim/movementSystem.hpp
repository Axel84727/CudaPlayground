#include "sim/ISystem.hpp"

class world;
class movementSystem : public ISystem
{
private:
    /* data */
    void verlet_integration(world &world);

public:
    void update(world &, float dt) override;
    movementSystem(/* args */);
    ~movementSystem();
};
