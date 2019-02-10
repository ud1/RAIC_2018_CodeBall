#ifndef RAIC2018_SIMULATOR_HPP
#define RAIC2018_SIMULATOR_HPP

#include "myutils3d.hpp"
#include <vector>
#include <optional>

constexpr F ROBOT_MIN_RADIUS = 1;
constexpr F ROBOT_MAX_RADIUS = 1.05;
constexpr F ROBOT_MAX_JUMP_SPEED = 15;
constexpr F ROBOT_ACCELERATION = 100;
constexpr F ROBOT_NITRO_ACCELERATION = 30;
constexpr F ROBOT_MAX_GROUND_SPEED = 30;
constexpr F ROBOT_ARENA_E = 0;
constexpr F ROBOT_RADIUS = 1;
constexpr F ROBOT_MASS = 2;
constexpr F TICKS_PER_SECOND = 60;
constexpr int TICKS_PER_SECOND_I = 60;
constexpr F TICK_DUR = 1.0 / TICKS_PER_SECOND;
constexpr int MICROTICKS_PER_TICK = 100;
constexpr F RESET_TICKS = 2 * TICKS_PER_SECOND;
constexpr F BALL_ARENA_E = 0.7;
constexpr F BALL_RADIUS = 2;
constexpr F BALL_MASS = 1;
constexpr F MIN_HIT_E = 0.4;
constexpr F MAX_HIT_E = 0.5;
constexpr F MAX_ENTITY_SPEED = 100;
constexpr F MAX_NITRO_AMOUNT = 100;
constexpr F START_NITRO_AMOUNT = 50;
constexpr F NITRO_POINT_VELOCITY_CHANGE = 0.6;
constexpr F NITRO_PACK_X = 20;
constexpr F NITRO_PACK_Y = 1;
constexpr F NITRO_PACK_Z = 30;
constexpr F NITRO_PACK_RADIUS = 0.5;
constexpr F NITRO_PACK_AMOUNT = 100;
constexpr int NITRO_RESPAWN_TICKS = 10 * TICKS_PER_SECOND_I;
constexpr F GRAVITY = 30;


constexpr F ARENA_WIDTH = 60;
constexpr F ARENA_HEIGHT = 20;
constexpr F ARENA_DEPTH = 80;
constexpr F ARENA_BOTTOM_RADIUS = 3;
constexpr F ARENA_TOP_RADIUS = 7;
constexpr F ARENA_CORNER_RADIUS = 13;
constexpr F ARENA_GOAL_TOP_RADIUS = 3;
constexpr F ARENA_GOAL_WIDTH = 30;
constexpr F ARENA_GOAL_HEIGHT = 10;
constexpr F ARENA_GOAL_DEPTH = 10;
constexpr F ARENA_GOAL_SIDE_RADIUS = 1;

struct Ball
{
    P3 pos;
    P3 vel;
    F mass;
    F rad;
    F radius_change_speed;
    F arena_e;
};

struct MyAction
{
    P3 target_velocity = P3(0);
    F jump_speed = 0;
    bool use_nitro = false;
};

enum class Side
{
    NONE = -1, NEG, POS
};

constexpr F getSideSign(Side side)
{
    if (side == Side::NEG)
        return 1;

    return -1;
}

struct MyRobot
{
    int id;
    Ball body;
    Side side;
    bool touch, ballTouch;
    P3 touch_normal;
    F nitro;

    MyAction action;
};

struct MyNitroPack
{
    P3 pos;
    F rad;
    int respawn_ticks;
};

struct MyArena
{
    F width;
    F height;
    F depth;
    F bottom_radius;
    F top_radius;
    F corner_radius;
    F goal_top_radius;
    F goal_width;
    F goal_height;
    F goal_depth;
    F goal_side_radius;
};

struct CollisionInfo
{
    P3 normal;
    F distance;

    void minTo(const CollisionInfo &oth)
    {
        if (oth.distance < distance)
        {
            *this = oth;
        }
    }
};

bool collide_entities(Ball &a, Ball &b);
CollisionInfo dan_to_arena(P3 point);

struct Simulator {
    Ball ball;
    std::vector<MyRobot> robots;
    std::vector<MyNitroPack> nitro_packs;
    const MyArena *arena;
    int curTick = 0;
    Side scoreSide = Side::NONE;

    void tick();
    void microtick();
    void tickFast();
    void tickFastUltra();
    void goal_scored();

    int indexOfRobot(int id) const {
        for (int i = 0; i < (int) robots.size(); ++i)
        {
            if (robots[i].id == id)
                return i;
        }

        return -1;
    }
};

void setFixedEInd(int ind);
void resetFixedEInd();


struct JumpAttrs
{
    F h;
    F v;
};

constexpr int NITRO_NUM = 4;
struct JumpConstants
{
    JumpConstants();

    std::vector<JumpAttrs> jumpAttrs[NITRO_NUM];
    std::vector<int> nitroTicks;
    std::vector<F> nitroConsumption;
};

extern JumpConstants JUMP_CONSTANTS;

#endif //RAIC2018_SIMULATOR_HPP
