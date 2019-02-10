#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if (curTick == -1)
    {
        arena = convertMyArena(rules);
    }

    if (curTick != game.current_tick)
    {
        Simulator sim = convertSimulator(&arena, game);
        strat.compute(sim);

        curTick = game.current_tick;
    }

    for (const MyRobot &r : strat.sim.robots)
    {
        if (r.side == Side::NEG && r.id == me.id)
        {
            action.target_velocity_x = r.action.target_velocity.x;
            action.target_velocity_y = r.action.target_velocity.y;
            action.target_velocity_z = r.action.target_velocity.z;
            action.jump_speed = r.action.jump_speed;
            action.use_nitro = r.action.use_nitro;

            LOG("ACTION " << r.id << " " << r.action.target_velocity << " " << r.action.jump_speed << " " << r.action.use_nitro);
            break;
        }
    }
}
