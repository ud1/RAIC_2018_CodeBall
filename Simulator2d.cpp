#include "Simulator2d.hpp"

void Robot2d::tick(P targetVel, bool inv) {
    targetVel = clampV(targetVel, ROBOT_MAX_GROUND_SPEED);
    P target_velocity_change = targetVel - vel;

    F l = target_velocity_change.len();
    P dir = (target_velocity_change).norm();

    if (l > ROBOT_ACCELERATION * TICK_DUR)
    {
        dir = dir * ROBOT_ACCELERATION * TICK_DUR;
        pos += vel * TICK_DUR + dir * (TICK_DUR * (inv ? 0.99 : 1.01)/2.0);
        vel += dir;
    }
    else
    {
        F k = ((int) (l * MICROTICKS_PER_TICK / (ROBOT_ACCELERATION * TICK_DUR)));
        dir = dir * l;
        F coef = ((1 + k)/(2.0 * MICROTICKS_PER_TICK) + 1 - k / MICROTICKS_PER_TICK);
        if (inv)
            coef = 1 - coef;
        pos += vel * TICK_DUR + dir * (TICK_DUR * coef);
        vel += dir;
    }
}

F computeDist(F vel, F targetVel, bool inv)
{
    F target_velocity_change = targetVel - vel;
    F l = std::abs(target_velocity_change);
    F dir = target_velocity_change > 0 ? 1 : -1;

    F pos = 0;
    if (l > ROBOT_ACCELERATION * TICK_DUR)
    {
        dir = dir * ROBOT_ACCELERATION * TICK_DUR;
        pos += vel * TICK_DUR + dir * (TICK_DUR * (inv ? 0.99 : 1.01)/2.0);
    }
    else
    {
        F k = ((int) (l * MICROTICKS_PER_TICK / (ROBOT_ACCELERATION * TICK_DUR)));
        dir = dir * l;
        F coef = ((1 + k)/(2.0 * MICROTICKS_PER_TICK) + 1 - k / MICROTICKS_PER_TICK);
        if (inv)
            coef = 1 - coef;
        pos += vel * TICK_DUR + dir * (TICK_DUR * coef);
    }

    return pos;
}

F optimizeSpeed(F deltaL, F curSpdF, F curSpdB, int T)
{
    F speed = deltaL / (T / TICKS_PER_SECOND);

    if (T > 2)
    {
        for (int k = 0; k < 20; ++k)
        {
            F newDist = deltaL - computeDist(curSpdF, speed, false) - computeDist(curSpdB, speed, true);
            F newSpeed = newDist / ((T - 2) / TICKS_PER_SECOND);
            speed = (speed + newSpeed)*0.5;
            if (std::abs(newSpeed - speed) < 0.001)
                break;
        }
    }

    return speed;
}

bool Trajectory2d::find(const P &targetPos, const P &targetVel, int targetT)
{
    P middle = (targetPos + robot2d.pos) * 0.5;
    F minDist = 1e6;

    for (int brakingTicks = 0; brakingTicks < targetT/2; ++brakingTicks)
    {
        for (int iter = 0; iter < (brakingTicks > 0 ? 1 : 2); ++ iter)
        {
            found = false;
            Robot2d robot2df = robot2d;
            Robot2d robot2db;
            robot2db.pos = targetPos;
            robot2db.vel = targetVel * (-1);

            std::vector<P> vfs;
            std::vector<P> vbs;

            F oldDist = robot2db.pos.dist(robot2df.pos);
            F speed = ROBOT_MAX_GROUND_SPEED;
            int remTicks = targetT;
            for (int t = 0; t < 100; ++t)
            {
                //LOG("PF " << robot2df.pos << " PB " << robot2db.pos << " VF " << robot2df.vel << " VB " << robot2db.vel);
                P dd = (robot2df.pos - robot2db.pos - robot2db.vel * TICK_DUR).norm();

                P vb;
                if (remTicks == 1)
                {
                    vb = (robot2df.pos - robot2db.pos) / TICK_DUR;
                }
                else
                {
                    if (iter > 0 && remTicks > targetT/2)
                    {
                        vb = (middle - robot2db.pos) / TICK_DUR;
                    }
                    else
                    {
                        F V = std::min((F) (robot2df.pos - robot2db.pos).len() / TICK_DUR, speed);
                        if (V > 1 && t < brakingTicks)
                        {
                            vb = (dd - robot2db.vel.norm()).norm() * V;
                            //vb = P(0, 0);
                        }
                        else
                        {
                            vb = dd * V;
                        }
                    }
                }

                vbs.push_back(robot2db.vel * (-1));
                robot2db.tickInv(vb);
                remTicks--;

                if (!remTicks)
                    break;

                dd = (robot2db.pos - robot2df.pos - robot2df.vel * TICK_DUR).norm();

                P vf;
                if (remTicks == 1)
                {
                    vf = (robot2db.pos - robot2df.pos) / TICK_DUR;
                }
                else
                {
                    if (iter > 0 && remTicks > targetT/2)
                    {
                        vf = (middle - robot2df.pos) / TICK_DUR;
                    }
                    else
                    {
                        F V = std::min((F) (robot2db.pos - robot2df.pos).len() / TICK_DUR, speed);
                        if (V > 1 && t < brakingTicks)
                        {
                            vf = (dd - robot2df.vel.norm()).norm() * V;
                            //vf = P(0, 0);
                        }
                        else
                        {
                            vf = dd * V;
                        }
                    }
                }

                robot2df.tick(vf);
                vfs.push_back(robot2df.vel);
                remTicks--;

                if (!remTicks)
                    break;

                P delta = robot2db.pos - robot2df.pos;
                F deltaL = delta.len();
                delta /= deltaL;

                if (deltaL < minDist)
                {
                    minDist = deltaL;
                    middle = (robot2db.pos + robot2df.pos) * 0.5;
                }

                F c1 = cross(delta, robot2df.vel);
                F c2 = cross(delta, robot2db.vel);

                if (std::abs(c1) < 10 && std::abs(c2) < 10)
                {
                    int T = targetT - (int) vfs.size() * 2;
                    if (T < 0)
                        break;

                    speed = optimizeSpeed(deltaL, dot(delta, robot2df.vel), -dot(delta, robot2db.vel), T);
                }

                F dist = robot2db.pos.dist(robot2df.pos);
                if (dist > oldDist)
                    break;

                oldDist = dist;
            }

            if ((robot2db.pos - robot2df.pos).len() < FEPS && (robot2db.vel + robot2df.vel).len() < FEPS)
            {
                vel.insert(vel.end(), vfs.begin(), vfs.end());
                vel.insert(vel.end(), vbs.rbegin(), vbs.rend());
                found = true;

                //if (iter > 0)
                //    LOG("FFFFFFFFFF");
                return true;
            }
        }
    }

    return false;
}

