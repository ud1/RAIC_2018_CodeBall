#ifndef RAIC2018_D_GAME_HPP
#define RAIC2018_D_GAME_HPP

#include "Simulator.hpp"
#include "d_tcpclient.hpp"
#include "d_format.hpp"

inline MyArena createArena()
{
    MyArena arena;

    arena.width = ARENA_WIDTH;
    arena.height = ARENA_HEIGHT;
    arena.depth = ARENA_DEPTH;
    arena.bottom_radius = ARENA_BOTTOM_RADIUS;
    arena.top_radius = ARENA_TOP_RADIUS;
    arena.corner_radius = ARENA_CORNER_RADIUS;
    arena.goal_top_radius = ARENA_GOAL_TOP_RADIUS;
    arena.goal_width = ARENA_GOAL_WIDTH;
    arena.goal_height = ARENA_GOAL_HEIGHT;
    arena.goal_depth = ARENA_GOAL_DEPTH;
    arena.goal_side_radius = ARENA_GOAL_SIDE_RADIUS;

    return arena;
}

inline MyRobot createRobot(int id, Side side, const P &pos)
{
    MyRobot r;

    r.id = id;

    r.body.pos = P3(pos.x, ROBOT_RADIUS, pos.z);
    r.body.vel = P3(0);
    r.body.mass = ROBOT_MASS;
    r.body.rad = ROBOT_RADIUS;
    r.body.radius_change_speed = 0;
    r.body.arena_e = ROBOT_ARENA_E;

    r.side = side;
    r.touch = true;
    r.touch_normal = P3(0, 1, 0);
    r.nitro = 0;

    return r;
}

inline F randRange(F minV, F maxV)
{
    return rand() / (F) RAND_MAX * (maxV - minV) + minV;
}

inline Simulator createSim(MyArena *arena, int robotsN, bool enableNitro)
{
    Simulator sim;
    sim.arena = arena;

    F alpha1 = randRange(0.5, M_PI * 0.5 - 0.1);
    F alpha2 = M_PI - randRange(0.5, M_PI * 0.5 - 0.1);

    F R = 20;

    F da = robotsN > 1 ? (alpha2 - alpha1) / (F) (robotsN - 1) : 0;
    int id = 0;
    for (int i = 0; i < robotsN; ++i)
    {
        F angle = alpha1 + i * da;

        P p = P(angle) * R;

        MyRobot rp = createRobot(++id, Side::POS, p);
        sim.robots.push_back(rp);

        MyRobot rn = createRobot(++id, Side::NEG, p * (-1));
        sim.robots.push_back(rn);
    }

    sim.ball.pos = P3(0, BALL_RADIUS * randRange(1, 4), 0);
    sim.ball.vel = P3(0, 0, 0);
    sim.ball.mass = BALL_MASS;
    sim.ball.rad = BALL_RADIUS;
    sim.ball.radius_change_speed = 0;
    sim.ball.arena_e = BALL_ARENA_E;

    sim.curTick = 0;

    if (enableNitro)
    {
        for (int i = 0; i < 4; ++i)
        {
            MyNitroPack pack;
            pack.respawn_ticks = 0;
            F x = NITRO_PACK_X * (i & 0x1 ? 1 : -1);
            F z = NITRO_PACK_Z * (i & 0x2 ? 1 : -1);
            pack.pos = P3(x, NITRO_PACK_Y, z);
            pack.rad = NITRO_PACK_RADIUS;

            sim.nitro_packs.push_back(pack);
        }

        for (MyRobot &r : sim.robots)
        {
            r.nitro = START_NITRO_AMOUNT;
        }
    }

    return sim;
}

inline Simulator mirrorSim(const Simulator &sim)
{
    Simulator res = sim;

    for (MyRobot &r : res.robots)
    {
        r.side = (Side) (1 - (int) r.side);
        r.body.pos.x *= -1;
        r.body.pos.z *= -1;
        r.body.vel.x *= -1;
        r.body.vel.z *= -1;
    }

    for (MyNitroPack &p : res.nitro_packs)
    {
        p.pos.x *= -1;
        p.pos.z *= -1;
    }

    res.ball.pos.x *= -1;
    res.ball.pos.z *= -1;
    res.ball.vel.x *= -1;
    res.ball.vel.z *= -1;

    return res;
}

inline void copyActions(Simulator &sim, Simulator &simFrom)
{
    for (MyRobot &r : simFrom.robots)
    {
        if (r.side == Side::NEG)
        {
            for (MyRobot &nr : sim.robots)
            {
                if (nr.id == r.id)
                {
                    assert(nr.side == Side::NEG);
                    nr.action = r.action;
                }
            }
        }
    }
}

inline void copyMirroredActions(Simulator &sim, Simulator &mirroredSim)
{
    for (MyRobot &r : mirroredSim.robots)
    {
        if (r.side == Side::NEG)
        {
            for (MyRobot &nr : sim.robots)
            {
                if (nr.id == r.id)
                {
                    assert(nr.side == Side::POS);
                    nr.action = r.action;
                    nr.action.target_velocity.x *= -1;
                    nr.action.target_velocity.z *= -1;
                }
            }
        }
    }
}

SObj sphere(const P3 &pos, float r, uint32_t color, const P3 &lightPos);
uint32_t rgbF(F r, F g, F b);
void sendNewTick(TcpClient &client, uint32_t tick);
void sendMap(TcpClient &tcpClient, const MyArena &arena);
void sendObjects(TcpClient &tcpClient, const Simulator &sim);

Obj saveSimAsObj(const Simulator &sim);
Simulator loadSimFromObj(MyArena *arena, const Obj &obj);

#endif //RAIC2018_D_GAME_HPP
