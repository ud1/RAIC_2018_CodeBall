#include "Simulator.hpp"
#include <algorithm>
#include <optional>
#include <cassert>

const static F FIXED_HIT_E[] = {0.41, 0.43, 0.45, 0.47, 0.49};
static int fixedHitEInd = -1;

void setFixedEInd(int ind)
{
    assert(ind >= 0 && ind < 5);
    fixedHitEInd = ind;
}

void resetFixedEInd()
{
    fixedHitEInd = -1;
}

bool collide_entities(Ball &a, Ball &b)
{
    P3 delta_position = b.pos - a.pos;
    F distance = length(delta_position);
    F penetration = a.rad + b.rad - distance;
    if (penetration > 0)
    {
        F k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass));
        F k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass));
        P3 normal = delta_position / distance;
        a.pos -= normal * penetration * k_a;
        b.pos += normal * penetration * k_b;
        F delta_velocity = dot(b.vel - a.vel, normal)
                - b.radius_change_speed - a.radius_change_speed;

        if (delta_velocity < 0)
        {
            F hitE = (1 + (fixedHitEInd >= 0 ? FIXED_HIT_E[fixedHitEInd] : random(MIN_HIT_E, MAX_HIT_E)));
            P3 impulse = hitE * delta_velocity * normal;
            a.vel += impulse * k_a;
            b.vel -= impulse * k_b;
        }

        return true;
    }

    return false;
}



CollisionInfo dan_to_arena(P3 point);

std::optional<P3> collide_with_arena(Ball &e)
{
    CollisionInfo colInfo = dan_to_arena(e.pos);
    F penetration = e.rad - colInfo.distance;

    if (penetration > 0)
    {
        e.pos += penetration * colInfo.normal;
        F velocity = dot(e.vel, colInfo.normal) - e.radius_change_speed;
        if (velocity < 0)
        {
            e.vel -= (1 + e.arena_e) * velocity * colInfo.normal;
            return colInfo.normal;
        }
    }

    return {};
}

template<int MICROTICS_NUM>
void move(Ball &e)
{
    constexpr F MICROTICK_DUR = TICK_DUR / MICROTICS_NUM;

    e.vel = clampV(e.vel, MAX_ENTITY_SPEED);
    e.pos += e.vel * MICROTICK_DUR;
    e.pos.y -= GRAVITY * MICROTICK_DUR * MICROTICK_DUR / 2;
    e.vel.y -= GRAVITY * MICROTICK_DUR;
}

template<int MICROTICS_NUM>
void update(Simulator &sim)
{
    //shuffle(robots);
    constexpr F MICROTICK_DUR = TICK_DUR / MICROTICS_NUM;

    for (MyRobot &robot : sim.robots)
    {
        if (robot.touch)
        {
            P3 target_velocity = clampV(
                    robot.action.target_velocity,
                    ROBOT_MAX_GROUND_SPEED);
            target_velocity -= robot.touch_normal
                    * dot(robot.touch_normal, target_velocity);

            P3 target_velocity_change = target_velocity - robot.body.vel;
            if (length(target_velocity_change) > 0)
            {
                F acceleration = ROBOT_ACCELERATION * maxF(0, robot.touch_normal.y);
                robot.body.vel += clampV(
                        target_velocity_change.norm() * acceleration * MICROTICK_DUR,
                        length(target_velocity_change));
            }
        }
        if (robot.action.use_nitro)
        {
            P3 target_velocity_change = clampV(
                    robot.action.target_velocity - robot.body.vel,
                    robot.nitro * NITRO_POINT_VELOCITY_CHANGE);
            if (length(target_velocity_change) > 0)
            {
                P3 acceleration = target_velocity_change.norm()
                                   * ROBOT_NITRO_ACCELERATION;
                P3 velocity_change = clampV(
                        acceleration * MICROTICK_DUR,
                        length(target_velocity_change));
                robot.body.vel += velocity_change;
                robot.nitro -= length(velocity_change) / NITRO_POINT_VELOCITY_CHANGE;

            }
        }

        move<MICROTICS_NUM>(robot.body);
        robot.body.rad = ROBOT_MIN_RADIUS + (ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS)
                                          * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
        robot.body.radius_change_speed = robot.action.jump_speed;
    }

    move<MICROTICS_NUM>(sim.ball);
    for (int i = 0; i < (int) sim.robots.size(); ++i)
    {
        for (int j = 0; j <= i - 1; ++j)
        {
            collide_entities(sim.robots[i].body, sim.robots[j].body);
        }
    }

    for (MyRobot &robot: sim.robots)
    {
        robot.ballTouch |= collide_entities(robot.body, sim.ball);
        auto collision_normal = collide_with_arena(robot.body);

        if (collision_normal)
        {
            robot.touch_normal = *collision_normal;
            robot.touch = true;
        }
        else
        {
            robot.touch = false;
        }
    }

    collide_with_arena(sim.ball);

    if (std::abs(sim.ball.pos.z) > ARENA_DEPTH / 2 + sim.ball.rad)
        sim.goal_scored();


    for (MyRobot &robot: sim.robots)
    {
        if (robot.nitro == MAX_NITRO_AMOUNT)
            continue;

        for (MyNitroPack &pack: sim.nitro_packs)
        {
            if (pack.respawn_ticks > 0)
                continue;

            if (length(robot.body.pos - pack.pos) <= robot.body.rad + pack.rad)
            {
                robot.nitro = MAX_NITRO_AMOUNT;
                pack.respawn_ticks = NITRO_RESPAWN_TICKS;
            }
        }
    }
}

void Simulator::goal_scored()
{
    if (ball.pos.z < 0)
        scoreSide = Side::POS;
    else
        scoreSide = Side::NEG;
}

void Simulator::tick()
{
    for (MyRobot &r : robots)
        r.ballTouch = false;

    for (int i = 0; i < MICROTICKS_PER_TICK; ++i)
        update<MICROTICKS_PER_TICK>(*this);

    for (MyNitroPack &pack: nitro_packs)
    {
        if (pack.respawn_ticks)
            pack.respawn_ticks -= 1;
    }

    ++curTick;
}

void Simulator::microtick()
{
    for (MyRobot &r : robots)
        r.ballTouch = false;

    update<MICROTICKS_PER_TICK>(*this);
}

void Simulator::tickFast() {
    for (MyRobot &r : robots)
        r.ballTouch = false;

    for (int i = 0; i < 20; ++i)
        update<20>(*this);

    for (MyNitroPack &pack: nitro_packs)
    {
        if (pack.respawn_ticks)
            pack.respawn_ticks -= 1;
    }

    ++curTick;
}

void Simulator::tickFastUltra() {
    for (MyRobot &r : robots)
        r.ballTouch = false;

    for (int i = 0; i < 5; ++i)
        update<5>(*this);

    for (MyNitroPack &pack: nitro_packs)
    {
        if (pack.respawn_ticks)
            pack.respawn_ticks -= 1;
    }

    ++curTick;
}


////////////////////////////////////////////////////////////////////

CollisionInfo dan_to_plane(const P3 &point, const P3 &point_on_plane, const P3 &plane_normal)
{
    CollisionInfo res;
    res.distance = dot(point - point_on_plane, plane_normal);
    res.normal = plane_normal;
    return res;
}

CollisionInfo dan_to_sphere_inner(const P3 &point, const P3 &sphere_center, F sphere_radius)
{
    CollisionInfo res;
    res.normal =  sphere_center - point;
    F len = res.normal.len();
    res.distance = sphere_radius - len;
    res.normal /= len;
    return res;
}

CollisionInfo dan_to_sphere_outer(const P3 &point, const P3 &sphere_center, F sphere_radius)
{
    CollisionInfo res;
    res.normal = point - sphere_center;
    F len = res.normal.len();
    res.distance = len - sphere_radius;
    res.normal /= len;
    return res;
}

CollisionInfo dan_to_arena_quarter(const P3 &point)
{
    // Ground
    CollisionInfo dan = dan_to_plane(point, P3(0, 0, 0), P3(0, 1, 0));
    // Ceiling
    dan.minTo(dan_to_plane(point, P3(0, ARENA_HEIGHT, 0), P3(0, -1, 0)));

    // Optimization
    if (point.x < 24 && point.z < 34 && point.y < 13)
        return dan;

    // Side x
    dan.minTo(dan_to_plane(point, P3(ARENA_WIDTH / 2, 0, 0), P3(-1, 0, 0)));

    // Side z (goal)
    dan.minTo(dan_to_plane(
            point,
            P3(0, 0, (ARENA_DEPTH / 2) + ARENA_GOAL_DEPTH),
            P3(0, 0, -1)));

    // Side z
    P v = P(point.x, point.y) - P(
            (ARENA_GOAL_WIDTH / 2) - ARENA_GOAL_TOP_RADIUS,
                    ARENA_GOAL_HEIGHT - ARENA_GOAL_TOP_RADIUS);

    if (point.x >= (ARENA_GOAL_WIDTH / 2) + ARENA_GOAL_SIDE_RADIUS
        || point.y >= ARENA_GOAL_HEIGHT + ARENA_GOAL_SIDE_RADIUS
        || (v.x > 0 && v.z > 0 && v.len() >= ARENA_GOAL_TOP_RADIUS + ARENA_GOAL_SIDE_RADIUS)
    )
    {
        dan.minTo(dan_to_plane(point, P3(0, 0, ARENA_DEPTH / 2), P3(0, 0, -1)));
    }

    // Side x & ceiling (goal)
    if (point.z >= (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS)
    {
        // x
        dan.minTo(dan_to_plane(
                point,
                P3(ARENA_GOAL_WIDTH / 2, 0, 0),
                P3(-1, 0, 0)));
        // y
        dan.minTo(dan_to_plane(point, P3(0, ARENA_GOAL_HEIGHT, 0), P3(0, -1, 0)));
    }

    // Goal back corners
    static_assert(ARENA_BOTTOM_RADIUS == ARENA_GOAL_TOP_RADIUS);

    if (point.z > (ARENA_DEPTH / 2) + ARENA_GOAL_DEPTH - ARENA_BOTTOM_RADIUS)
    {
        dan.minTo(dan_to_sphere_inner(
            point,
            P3(
                clamp(
                        point.x,
                        ARENA_BOTTOM_RADIUS - (ARENA_GOAL_WIDTH / 2),
                        (ARENA_GOAL_WIDTH / 2) - ARENA_BOTTOM_RADIUS
                ),
                clamp(
                        point.y,
                        ARENA_BOTTOM_RADIUS,
                        ARENA_GOAL_HEIGHT - ARENA_GOAL_TOP_RADIUS
                ),
                (ARENA_DEPTH / 2) + ARENA_GOAL_DEPTH - ARENA_BOTTOM_RADIUS),
            ARENA_BOTTOM_RADIUS));
    }

    // Corner
    if (point.x > (ARENA_WIDTH / 2) - ARENA_CORNER_RADIUS &&
        point.z > (ARENA_DEPTH / 2) - ARENA_CORNER_RADIUS)
    {
        dan.minTo(dan_to_sphere_inner(
                point,
                P3(
                        (ARENA_WIDTH / 2) - ARENA_CORNER_RADIUS,
                        point.y,
                        (ARENA_DEPTH / 2) - ARENA_CORNER_RADIUS
                ),
        ARENA_CORNER_RADIUS));
    }

    // Goal outer corner
    if (point.z < (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS)
    {
        // Side x
        if (point.x < (ARENA_GOAL_WIDTH / 2) + ARENA_GOAL_SIDE_RADIUS)
        {
            dan.minTo(dan_to_sphere_outer(
                point,
                P3(
                        (ARENA_GOAL_WIDTH / 2) + ARENA_GOAL_SIDE_RADIUS,
                                point.y,
                                (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS
                ),
                ARENA_GOAL_SIDE_RADIUS));
        }

        // Ceiling
        if (point.y < ARENA_GOAL_HEIGHT + ARENA_GOAL_SIDE_RADIUS)
        {
            dan.minTo(dan_to_sphere_outer(
                point,
                P3(
                        point.x,
                                ARENA_GOAL_HEIGHT + ARENA_GOAL_SIDE_RADIUS,
                                (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS
                ),
                ARENA_GOAL_SIDE_RADIUS));
        }

        // Top corner
        P o = P(
                (ARENA_GOAL_WIDTH / 2) - ARENA_GOAL_TOP_RADIUS,
                        ARENA_GOAL_HEIGHT - ARENA_GOAL_TOP_RADIUS
        );

        P v = P(point.x, point.y) - o;
        if (v.x > 0 and v.z > 0)
        {
            o += v.norm() * (ARENA_GOAL_TOP_RADIUS + ARENA_GOAL_SIDE_RADIUS);
            dan.minTo(dan_to_sphere_outer(
                    point,
                    P3(o.x, o.z, (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS),
                    ARENA_GOAL_SIDE_RADIUS));
        }
    }

    // Goal inside top corners
    if (point.z > (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS &&
            point.y > ARENA_GOAL_HEIGHT - ARENA_GOAL_TOP_RADIUS)
    {
        // Side x
        if (point.x > (ARENA_GOAL_WIDTH / 2) - ARENA_GOAL_TOP_RADIUS)
        {
            dan.minTo(dan_to_sphere_inner(
                    point,
                    P3(
                            (ARENA_GOAL_WIDTH / 2) - ARENA_GOAL_TOP_RADIUS,
                                    ARENA_GOAL_HEIGHT - ARENA_GOAL_TOP_RADIUS,
                                    point.z
                    ),
                    ARENA_GOAL_TOP_RADIUS));
        }

        // Side z
        if (point.z > (ARENA_DEPTH / 2) + ARENA_GOAL_DEPTH - ARENA_GOAL_TOP_RADIUS)
        {
            dan.minTo(dan_to_sphere_inner(
                    point,
                    P3(
                            point.x,
                            ARENA_GOAL_HEIGHT - ARENA_GOAL_TOP_RADIUS,
                            (ARENA_DEPTH / 2) + ARENA_GOAL_DEPTH - ARENA_GOAL_TOP_RADIUS
                     ),
                     ARENA_GOAL_TOP_RADIUS));
        }
    }

    // Bottom corners
    if (point.y < ARENA_BOTTOM_RADIUS)
    {
        // Side x
        if (point.x > (ARENA_WIDTH / 2) - ARENA_BOTTOM_RADIUS)
        {
            dan.minTo(dan_to_sphere_inner(
                point,
                P3(
                        (ARENA_WIDTH / 2) - ARENA_BOTTOM_RADIUS,
                                ARENA_BOTTOM_RADIUS,
                                point.z
                ),
                ARENA_BOTTOM_RADIUS));
        }

        // Side z
        if (point.z > (ARENA_DEPTH / 2) - ARENA_BOTTOM_RADIUS
                and point.x >= (ARENA_GOAL_WIDTH / 2) + ARENA_GOAL_SIDE_RADIUS)
        {
            dan.minTo(dan_to_sphere_inner(
                point,
                P3(
                        point.x,
                                ARENA_BOTTOM_RADIUS,
                                (ARENA_DEPTH / 2) - ARENA_BOTTOM_RADIUS
                ),
                ARENA_BOTTOM_RADIUS));
        }

        // Side z (goal)
        if (point.z > (ARENA_DEPTH / 2) + ARENA_GOAL_DEPTH - ARENA_BOTTOM_RADIUS)
        {
            dan.minTo(dan_to_sphere_inner(
                point,
                P3(
                        point.x,
                                ARENA_BOTTOM_RADIUS,
                                (ARENA_DEPTH / 2) + ARENA_GOAL_DEPTH - ARENA_BOTTOM_RADIUS
                ),
                ARENA_BOTTOM_RADIUS));
        }

        // Goal outer corner
        P o = P(
                (ARENA_GOAL_WIDTH / 2) + ARENA_GOAL_SIDE_RADIUS,
                        (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS
        );
        P v = P(point.x, point.z) - o;
        if (v.x < 0 and v.z < 0 &&
            v.len() < ARENA_GOAL_SIDE_RADIUS + ARENA_BOTTOM_RADIUS)
        {
            o += v.norm() * (ARENA_GOAL_SIDE_RADIUS + ARENA_BOTTOM_RADIUS);
            dan.minTo(dan_to_sphere_inner(
                point,
                P3(o.x, ARENA_BOTTOM_RADIUS, o.z),
                ARENA_BOTTOM_RADIUS));
        }

        // Side x (goal)
        if (point.z >= (ARENA_DEPTH / 2) + ARENA_GOAL_SIDE_RADIUS &&
                point.x > (ARENA_GOAL_WIDTH / 2) - ARENA_BOTTOM_RADIUS)
        {
            dan.minTo(dan_to_sphere_inner(

            point,
            P3(
            (ARENA_GOAL_WIDTH / 2) - ARENA_BOTTOM_RADIUS,
            ARENA_BOTTOM_RADIUS,
            point.z
            ),
            ARENA_BOTTOM_RADIUS));
        }
        // Corner
        if (point.x > (ARENA_WIDTH / 2) - ARENA_CORNER_RADIUS
                and point.z > (ARENA_DEPTH / 2) - ARENA_CORNER_RADIUS)
        {
            P corner_o = P(
                    (ARENA_WIDTH / 2) - ARENA_CORNER_RADIUS,
                            (ARENA_DEPTH / 2) - ARENA_CORNER_RADIUS
            );
            P n = P(point.x, point.z) - corner_o;
            F dist = n.len();
            if (dist > ARENA_CORNER_RADIUS - ARENA_BOTTOM_RADIUS)
            {
                n /= dist;
                P o2 = corner_o + n * (ARENA_CORNER_RADIUS - ARENA_BOTTOM_RADIUS);
                dan.minTo(dan_to_sphere_inner(
                        point,
                        P3(o2.x, ARENA_BOTTOM_RADIUS, o2.z),
                        ARENA_BOTTOM_RADIUS));
            }
        }
    }

    // Ceiling corners
    if (point.y > ARENA_HEIGHT - ARENA_TOP_RADIUS)
    {
        // Side x
        if (point.x > (ARENA_WIDTH / 2) - ARENA_TOP_RADIUS)
        {
        dan.minTo(dan_to_sphere_inner(
                point,
                P3(
                        (ARENA_WIDTH / 2) - ARENA_TOP_RADIUS,
                                ARENA_HEIGHT - ARENA_TOP_RADIUS,
                                point.z
                ),
                ARENA_TOP_RADIUS));
        }
        
        // Side z
        if (point.z > (ARENA_DEPTH / 2) - ARENA_TOP_RADIUS)
        {
            dan.minTo(dan_to_sphere_inner(
                    point,
                    P3(
                            point.x,
                                    ARENA_HEIGHT - ARENA_TOP_RADIUS,
                                    (ARENA_DEPTH / 2) - ARENA_TOP_RADIUS
                    ),
            ARENA_TOP_RADIUS));
        }
        // Corner
        if (point.x > (ARENA_WIDTH / 2) - ARENA_CORNER_RADIUS
                and point.z > (ARENA_DEPTH / 2) - ARENA_CORNER_RADIUS)
        {
            P corner_o = P(
                    (ARENA_WIDTH / 2) - ARENA_CORNER_RADIUS,
                            (ARENA_DEPTH / 2) - ARENA_CORNER_RADIUS
            );
            P dv = P(point.x, point.z) - corner_o;
            if (dv.len() > ARENA_CORNER_RADIUS - ARENA_TOP_RADIUS)
            {
                P n = dv.norm();
                P o2 = corner_o + n * (ARENA_CORNER_RADIUS - ARENA_TOP_RADIUS);
                dan.minTo(dan_to_sphere_inner(
                        point,
                        P3(o2.x, ARENA_HEIGHT - ARENA_TOP_RADIUS, o2.z),
                        ARENA_TOP_RADIUS));
            }
        }
    }
    return dan;
}

CollisionInfo dan_to_arena(P3 point)
{
    bool negate_x = point.x < 0;
    bool negate_z = point.z < 0;
    if (negate_x)
        point.x = -point.x;

    if (negate_z)
        point.z = -point.z;

    CollisionInfo result = dan_to_arena_quarter(point);
    if (negate_x)
        result.normal.x = -result.normal.x;

    if (negate_z)
        result.normal.z = -result.normal.z;

    return result;
}

JumpConstants JUMP_CONSTANTS;
JumpConstants::JumpConstants()
{
    nitroTicks.push_back(0);
    nitroConsumption.push_back(0);
    jumpAttrs[0].push_back({1.000000, 0.000000});
    jumpAttrs[0].push_back({1.293147, 14.488322});
    jumpAttrs[0].push_back({1.530458, 13.988311});
    jumpAttrs[0].push_back({1.759436, 13.488299});
    jumpAttrs[0].push_back({1.980080, 12.988288});
    jumpAttrs[0].push_back({2.192381, 12.488276});
    jumpAttrs[0].push_back({2.396346, 11.988265});
    jumpAttrs[0].push_back({2.591978, 11.488254});
    jumpAttrs[0].push_back({2.779276, 10.988242});
    jumpAttrs[0].push_back({2.958241, 10.488231});
    jumpAttrs[0].push_back({3.128872, 9.988219});
    jumpAttrs[0].push_back({3.291169, 9.488208});
    jumpAttrs[0].push_back({3.445133, 8.988196});
    jumpAttrs[0].push_back({3.590764, 8.488185});
    jumpAttrs[0].push_back({3.728061, 7.988173});
    jumpAttrs[0].push_back({3.857024, 7.488162});
    jumpAttrs[0].push_back({3.977654, 6.988151});
    jumpAttrs[0].push_back({4.089950, 6.488139});
    jumpAttrs[0].push_back({4.193912, 5.988128});
    jumpAttrs[0].push_back({4.289542, 5.488116});
    jumpAttrs[0].push_back({4.376838, 4.988105});
    jumpAttrs[0].push_back({4.455800, 4.488093});
    jumpAttrs[0].push_back({4.526429, 3.988082});
    jumpAttrs[0].push_back({4.588725, 3.488070});
    jumpAttrs[0].push_back({4.642686, 2.988059});
    jumpAttrs[0].push_back({4.688314, 2.488048});
    jumpAttrs[0].push_back({4.725609, 1.988037});
    jumpAttrs[0].push_back({4.754570, 1.488037});
    jumpAttrs[0].push_back({4.775198, 0.988037});
    jumpAttrs[0].push_back({4.787493, 0.488038});

    nitroTicks.push_back(8);
    nitroConsumption.push_back(7);
    jumpAttrs[1].push_back({1.000000, 0.000000});
    jumpAttrs[1].push_back({1.297270, 14.983334});
    jumpAttrs[1].push_back({1.547037, 14.983334});
    jumpAttrs[1].push_back({1.796804, 14.983334});
    jumpAttrs[1].push_back({2.046571, 14.983334});
    jumpAttrs[1].push_back({2.296339, 14.983334});
    jumpAttrs[1].push_back({2.546106, 14.983334});
    jumpAttrs[1].push_back({2.795873, 14.983334});
    jumpAttrs[1].push_back({3.045641, 14.983334});
    jumpAttrs[1].push_back({3.291190, 14.483322});
    jumpAttrs[1].push_back({3.528406, 13.983311});
    jumpAttrs[1].push_back({3.757288, 13.483299});
    jumpAttrs[1].push_back({3.977837, 12.983288});
    jumpAttrs[1].push_back({4.190053, 12.483276});
    jumpAttrs[1].push_back({4.393935, 11.983265});
    jumpAttrs[1].push_back({4.589484, 11.483253});
    jumpAttrs[1].push_back({4.776699, 10.983242});
    jumpAttrs[1].push_back({4.955581, 10.483231});
    jumpAttrs[1].push_back({5.126129, 9.983219});
    jumpAttrs[1].push_back({5.288343, 9.483208});
    jumpAttrs[1].push_back({5.442224, 8.983196});
    jumpAttrs[1].push_back({5.587770, 8.483185});
    jumpAttrs[1].push_back({5.724984, 7.983173});
    jumpAttrs[1].push_back({5.853863, 7.483162});
    jumpAttrs[1].push_back({5.974410, 6.983150});
    jumpAttrs[1].push_back({6.086622, 6.483139});
    jumpAttrs[1].push_back({6.190501, 5.983128});
    jumpAttrs[1].push_back({6.286047, 5.483116});
    jumpAttrs[1].push_back({6.373260, 4.983105});
    jumpAttrs[1].push_back({6.452139, 4.483093});
    jumpAttrs[1].push_back({6.522685, 3.983082});
    jumpAttrs[1].push_back({6.584897, 3.483070});
    jumpAttrs[1].push_back({6.638775, 2.983059});
    jumpAttrs[1].push_back({6.684320, 2.483047});
    jumpAttrs[1].push_back({6.721532, 1.983037});
    jumpAttrs[1].push_back({6.750410, 1.483037});
    jumpAttrs[1].push_back({6.770954, 0.983037});
    jumpAttrs[1].push_back({6.783165, 0.483038});

    nitroTicks.push_back(15);
    nitroConsumption.push_back(13);
    jumpAttrs[2].push_back({1.000000, 0.000000});
    jumpAttrs[2].push_back({1.297270, 14.983334});
    jumpAttrs[2].push_back({1.547037, 14.983334});
    jumpAttrs[2].push_back({1.796804, 14.983334});
    jumpAttrs[2].push_back({2.046571, 14.983334});
    jumpAttrs[2].push_back({2.296339, 14.983334});
    jumpAttrs[2].push_back({2.546106, 14.983334});
    jumpAttrs[2].push_back({2.795873, 14.983334});
    jumpAttrs[2].push_back({3.045641, 14.983334});
    jumpAttrs[2].push_back({3.295408, 14.983334});
    jumpAttrs[2].push_back({3.545175, 14.983334});
    jumpAttrs[2].push_back({3.794943, 14.983334});
    jumpAttrs[2].push_back({4.044710, 14.983334});
    jumpAttrs[2].push_back({4.294477, 14.983334});
    jumpAttrs[2].push_back({4.544244, 14.983334});
    jumpAttrs[2].push_back({4.794012, 14.983334});
    jumpAttrs[2].push_back({5.039561, 14.483322});
    jumpAttrs[2].push_back({5.276776, 13.983311});
    jumpAttrs[2].push_back({5.505659, 13.483299});
    jumpAttrs[2].push_back({5.726208, 12.983288});
    jumpAttrs[2].push_back({5.938424, 12.483276});
    jumpAttrs[2].push_back({6.142306, 11.983265});
    jumpAttrs[2].push_back({6.337854, 11.483253});
    jumpAttrs[2].push_back({6.525070, 10.983242});
    jumpAttrs[2].push_back({6.703951, 10.483231});
    jumpAttrs[2].push_back({6.874499, 9.983219});
    jumpAttrs[2].push_back({7.036714, 9.483208});
    jumpAttrs[2].push_back({7.190595, 8.983196});
    jumpAttrs[2].push_back({7.336142, 8.483185});
    jumpAttrs[2].push_back({7.473355, 7.983173});
    jumpAttrs[2].push_back({7.602234, 7.483162});
    jumpAttrs[2].push_back({7.722781, 6.983150});
    jumpAttrs[2].push_back({7.834993, 6.483139});
    jumpAttrs[2].push_back({7.938872, 5.983128});
    jumpAttrs[2].push_back({8.034437, 5.483116});
    jumpAttrs[2].push_back({8.121697, 4.983105});
    jumpAttrs[2].push_back({8.200624, 4.483093});
    jumpAttrs[2].push_back({8.271218, 3.983082});
    jumpAttrs[2].push_back({8.333478, 3.483070});
    jumpAttrs[2].push_back({8.387404, 2.983059});
    jumpAttrs[2].push_back({8.432998, 2.483047});
    jumpAttrs[2].push_back({8.470256, 1.983037});
    jumpAttrs[2].push_back({8.499183, 1.483037});
    jumpAttrs[2].push_back({8.519774, 0.983037});
    jumpAttrs[2].push_back({8.532033, 0.483038});

    nitroTicks.push_back(22);
    nitroConsumption.push_back(19);
    jumpAttrs[3].push_back({1.000000, 0.000000});
    jumpAttrs[3].push_back({1.297266, 14.983333});
    jumpAttrs[3].push_back({1.547030, 14.983333});
    jumpAttrs[3].push_back({1.796794, 14.983333});
    jumpAttrs[3].push_back({2.046558, 14.983333});
    jumpAttrs[3].push_back({2.296322, 14.983333});
    jumpAttrs[3].push_back({2.546086, 14.983333});
    jumpAttrs[3].push_back({2.795850, 14.983333});
    jumpAttrs[3].push_back({3.045613, 14.983333});
    jumpAttrs[3].push_back({3.295377, 14.983333});
    jumpAttrs[3].push_back({3.545141, 14.983333});
    jumpAttrs[3].push_back({3.794905, 14.983333});
    jumpAttrs[3].push_back({4.044669, 14.983333});
    jumpAttrs[3].push_back({4.294433, 14.983333});
    jumpAttrs[3].push_back({4.544197, 14.983333});
    jumpAttrs[3].push_back({4.793961, 14.983333});
    jumpAttrs[3].push_back({5.043725, 14.983333});
    jumpAttrs[3].push_back({5.293488, 14.983333});
    jumpAttrs[3].push_back({5.543252, 14.983333});
    jumpAttrs[3].push_back({5.793016, 14.983333});
    jumpAttrs[3].push_back({6.042780, 14.983333});
    jumpAttrs[3].push_back({6.292544, 14.983333});
    jumpAttrs[3].push_back({6.542308, 14.983333});
    jumpAttrs[3].push_back({6.787863, 14.483333});
    jumpAttrs[3].push_back({7.025086, 13.983333});
    jumpAttrs[3].push_back({7.253975, 13.483333});
    jumpAttrs[3].push_back({7.474530, 12.983333});
    jumpAttrs[3].push_back({7.686752, 12.483333});
    jumpAttrs[3].push_back({7.890641, 11.983333});
    jumpAttrs[3].push_back({8.086197, 11.483333});
    jumpAttrs[3].push_back({8.273419, 10.983333});
    jumpAttrs[3].push_back({8.452308, 10.483333});
    jumpAttrs[3].push_back({8.622863, 9.983333});
    jumpAttrs[3].push_back({8.785086, 9.483333});
    jumpAttrs[3].push_back({8.938975, 8.983333});
    jumpAttrs[3].push_back({9.084530, 8.483333});
    jumpAttrs[3].push_back({9.221752, 7.983333});
    jumpAttrs[3].push_back({9.350641, 7.483333});
    jumpAttrs[3].push_back({9.471197, 6.983333});
    jumpAttrs[3].push_back({9.583419, 6.483333});
    jumpAttrs[3].push_back({9.687308, 5.983333});
    jumpAttrs[3].push_back({9.782863, 5.483333});
    jumpAttrs[3].push_back({9.870086, 4.983333});
    jumpAttrs[3].push_back({9.948975, 4.483333});
    jumpAttrs[3].push_back({10.019530, 3.983333});
    jumpAttrs[3].push_back({10.081752, 3.483333});
    jumpAttrs[3].push_back({10.135641, 2.983333});
    jumpAttrs[3].push_back({10.181197, 2.483333});
    jumpAttrs[3].push_back({10.218419, 1.983333});
    jumpAttrs[3].push_back({10.247308, 1.483333});
    jumpAttrs[3].push_back({10.267863, 0.983333});
    jumpAttrs[3].push_back({10.280086, 0.483333});
}
