#ifndef RAIC2018_SIMULATOR2D_HPP
#define RAIC2018_SIMULATOR2D_HPP

#include "Simulator.hpp"

struct Robot2d
{
    P pos;
    P vel;

    Robot2d(){}

    Robot2d(const MyRobot &r) {
        vel = r.body.vel.xz();
        pos = r.body.pos.xz();
    }

    void tick(P targetVel)
    {
        tick(targetVel, false);
    }

    void tickInv(P targetVel)
    {
        tick(targetVel, true);
    }

private:
    void tick(P targetVel, bool inv);
};

F computeDist(F vel, F targetVel, bool inv);

struct Trajectory2d
{
    Robot2d robot2d;
    std::vector<P> vel;
    int flyT;
    int nitroTicks = 0;
    bool found = false;
    P lastVel;

    bool find(const P &targetPos, const P &targetVel, int targetT);
    int len() const
    {
        return (int) vel.size() + flyT;
    }
};

#endif //RAIC2018_SIMULATOR2D_HPP
