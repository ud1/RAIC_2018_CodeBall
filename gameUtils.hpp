#ifndef RAIC2018_GAMEUTILS_HPP
#define RAIC2018_GAMEUTILS_HPP

#include "Simulator.hpp"
#include "Strategy.h"

inline MyArena convertMyArena(const model::Rules &rules)
{
    MyArena res;

    res.width = rules.arena.width;
    res.height = rules.arena.height;
    res.depth = rules.arena.depth;
    res.bottom_radius = rules.arena.bottom_radius;
    res.top_radius = rules.arena.top_radius;
    res.corner_radius = rules.arena.corner_radius;
    res.goal_top_radius = rules.arena.goal_top_radius;
    res.goal_width = rules.arena.goal_width;
    res.goal_height = rules.arena.goal_height;
    res.goal_depth = rules.arena.goal_depth;
    res.goal_side_radius = rules.arena.goal_side_radius;

    return res;
}

inline Simulator convertSimulator(const MyArena *arena, const model::Game& game)
{
    Simulator sim;
    sim.arena = arena;

    sim.curTick = game.current_tick;

    sim.ball.pos = P3(game.ball.x, game.ball.y, game.ball.z);
    sim.ball.vel = P3(game.ball.velocity_x, game.ball.velocity_y, game.ball.velocity_z);
    sim.ball.rad = game.ball.radius;
    sim.ball.mass = BALL_MASS;
    sim.ball.radius_change_speed = 0;
    sim.ball.arena_e = BALL_ARENA_E;

    for (const model::Robot &r : game.robots)
    {
        MyRobot robot;
        robot.id = r.id;
        robot.body.pos = P3(r.x, r.y, r.z);
        robot.body.vel = P3(r.velocity_x, r.velocity_y, r.velocity_z);
        robot.body.rad = r.radius;
        robot.body.mass = ROBOT_MASS;
        robot.body.radius_change_speed = 0; // TODO ????
        robot.body.arena_e = ROBOT_ARENA_E;

        robot.side = r.is_teammate ? Side::NEG : Side::POS;
        robot.touch = r.touch;
        robot.touch_normal = P3(r.touch_normal_x, r.touch_normal_y, r.touch_normal_z);
        robot.nitro = r.nitro_amount;

        sim.robots.push_back(robot);
    }

    for (const model::NitroPack &p : game.nitro_packs)
    {
        MyNitroPack nitro;
        nitro.pos = P3(p.x, p.y, p.z);
        nitro.rad = p.radius;

        if (p.alive)
            nitro.respawn_ticks = 0;
        else
            nitro.respawn_ticks = p.respawn_ticks;

        sim.nitro_packs.push_back(nitro);
    }

    return sim;
}

#endif //RAIC2018_GAMEUTILS_HPP
