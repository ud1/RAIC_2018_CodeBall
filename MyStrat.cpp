#include "MyStrat.hpp"
#include <vector>
#include <map>

constexpr size_t PRED_COUNT = 90;
constexpr F NITRO_POINTS = 100;

inline bool testDist(const P &delta, size_t ticks)
{
    return delta.len2() < sqr(ticks * ROBOT_MAX_GROUND_SPEED * TICK_DUR);
}

struct PredictedBallIterator
{
    Simulator sim;
    int t = 0;
    int myId;

    PredictedBallIterator(const Simulator &_sim, int myId)
    {
        sim = _sim;
        filterVector(sim.robots, [myId](const MyRobot &r){return r.id == myId;});
        this->myId = myId;
    }

    bool computeTick(std::map<int, RobotState> &robotState)
    {
        if (t > 20)
        {
            sim.robots.clear();
        }
        else
        {
            for (MyRobot &r : sim.robots)
            {
                r.action.jump_speed = 0;
                r.action.use_nitro = false;
                r.action.target_velocity = xz((sim.ball.pos - r.body.pos).xz(), 0) / TICK_DUR;
                if ((!r.touch && r.body.vel.y > 1) || (sim.ball.pos - r.body.pos).len2() < sqr(BALL_RADIUS + ROBOT_MAX_RADIUS - 0.01))
                {
                    r.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                }
                else if (r.side == Side::POS)
                {
                    const RobotState &st = robotState[r.id];
                    if (st.jumpT >= 0 && st.jumpT <= t)
                    {
                        r.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                    }
                }
            }
        }

        sim.tickFast();

        t++;

        if (myId % 2 == 0)
            renderSphere(sim.ball.pos, 0.1, 0xffff00ffu);

        return finished();
    }

    bool finished() const
    {
        return (sim.scoreSide == Side::NONE);
    }
};

struct EngagePoint {
    EngagePoint(const Simulator &sim, const MyRobot &me) : predictedBallIterator(sim, me.id)
    {
    }

    P3 foundTargetPos;
    F height;
    size_t targetT;
    size_t arrivalT;
    bool found;
    P arrivalSpd;

    PredictedBallIterator predictedBallIterator;
};

P3 getTargetPos(const Ball &ball, const MyRobot &me);

EngagePoint computeEngagePoint(const Simulator &sim, const MyRobot &me, int count, bool highDanger, std::map<int, RobotState> &robotState)
{
    EngagePoint result = EngagePoint(sim, me);

    result.foundTargetPos = getTargetPos(sim.ball, me);
    result.height = ROBOT_RADIUS;

    result.targetT = PRED_COUNT - 1;
    result.arrivalT = PRED_COUNT - 1;

    result.found = false;

    for (size_t i = 0; i < count; )
    {
        if (i > 0)
        {
            if (!result.predictedBallIterator.computeTick(robotState))
                break;
        }

        Ball &ball = result.predictedBallIterator.sim.ball;

        /*if (enEngageT >= 0 && !result.found && i == enEngageT)
        {
            result.foundTargetPos = getTargetPos(ball, me);
        }*/
        //renderSphere(ball.pos, 0.02, 0xff000000);

        F maxHeight = me.nitro > 15 ? 8.5 : 6.5;
        if (highDanger)
        {
            if (me.nitro > 20)
                maxHeight = 11;
            else if (me.nitro > 15)
                maxHeight = 9.5;
            else
                maxHeight = 7;
        }

        if (ball.pos.y < maxHeight || (ball.pos.y < 9 && ball.pos.z < (-ARENA_DEPTH * 0.5)))
        {
            P3 targetPos = getTargetPos(ball, me);

            if (testDist(targetPos.xz() - me.body.pos.xz(), i + 1))
            {
                Robot2d robot2d = Robot2d(me);

                for (size_t t = 0; t <= i; ++t)
                {
                    F remainingDist = (robot2d.pos - targetPos.xz()).len();
                    if (remainingDist < robot2d.vel.len() * TICK_DUR)
                    {
                        result.foundTargetPos = targetPos;
                        result.height = ball.pos.y;
                        result.targetT = i;
                        result.found = true;
                        result.arrivalT = t;
                        result.arrivalSpd = robot2d.vel;
                        //LOG("FOUND " << i << " " << foundTargetPos);
                        break;
                    }

                    robot2d.tick((targetPos.xz() - robot2d.pos)/TICK_DUR);
                }
            }
        }

        if (result.found)
            break;

        ++i;
    }

    return result;
}

F ballScoreFast(const Simulator &ssim, const std::vector<MyRobot> &robots)
{
    const Ball &ball = ssim.ball;
    F sscore = ball.vel.z;

    if (ball.vel.z == 0)
        return 0;


    F T = ball.vel.z > 0 ? (ARENA_DEPTH * 0.5 + BALL_RADIUS - ball.pos.z) / ball.vel.z : (-(ARENA_DEPTH * 0.5 + BALL_RADIUS) - ball.pos.z) / ball.vel.z;

    F x = ball.pos.x + ball.vel.x * T;
    /*if (ball.vel.z > 0 && std::abs(x) < ARENA_WIDTH * 0.4)
    {
        F y = ball.pos.y + ball.vel.y * T - GRAVITY * 0.5 * sqr(T);

        if (y > 5 && y < 10)
        {
            sscore += 50;
        }
    }*/

    if (std::abs(x) < (ARENA_GOAL_WIDTH * 0.5 - BALL_RADIUS * 0.7))
    {
        F score;
        F y = ball.pos.y + ball.vel.y * T - GRAVITY * 0.5 * sqr(T);
        if (y > 0 && y < (ARENA_GOAL_HEIGHT - BALL_RADIUS * 0.7))
        {
            if (y > 5)
            {
                score = 1500;
            }
            else
                score = 1000;
        }
        else
        {
            score = 200;
        }

        if (ball.vel.z > 0)
        {
            if (score > 500)
            {
                bool myRobotNearby = false;
                bool enRobotNearby = false;
                for (const MyRobot & r : robots)
                {
                    if (!myRobotNearby && r.side == Side::NEG)
                    {
                        if ((ball.pos.xz() + ball.vel.xz() * T).dist(r.body.pos.xz()) < 30 * T)
                        {
                            myRobotNearby = true;
                        }
                    }

                    if (!enRobotNearby && r.side == Side::POS)
                    {
                        if ((ball.pos.xz() + ball.vel.xz() * T).dist(r.body.pos.xz()) < 30 * T)
                        {
                            enRobotNearby = true;
                        }
                    }
                }

                if (!myRobotNearby && enRobotNearby)
                {
                    score *= 0.8;
                }
            }
            sscore += score;
        }
        else
        {
            sscore -= score;
        }
    }
    else
    {
        if (ball.vel.z > 0 && ball.vel.x != 0)
        {
            F vx = ball.vel.x;
            F px = ball.pos.x;

            if (px < 0)
            {
                px *= -1;
                vx *= -1;
            }

            F wallT = (vx > 0) ? (ARENA_WIDTH * 0.5 - px) / vx : (ARENA_WIDTH * 0.5 + px) / (-vx);
            if (wallT < T)
            {
                F y = ball.pos.y + ball.vel.y * wallT - GRAVITY * 0.5 * sqr(wallT);
                if (y > 3 && y < ARENA_HEIGHT - 7)
                {
                    F tx = ARENA_WIDTH * 0.5 - std::abs(vx) * BALL_ARENA_E * (T - wallT);
                    if (std::abs(tx) < (ARENA_GOAL_WIDTH * 0.5 - BALL_RADIUS * 0.7))
                    {
                        sscore += 500;
                        F y = ball.pos.y + ball.vel.y * T - GRAVITY * 0.5 * sqr(T);
                        if (y > 5 && y < (ARENA_GOAL_HEIGHT - BALL_RADIUS * 0.7))
                        {
                            sscore += 200;
                        }
                    }
                }
            }
        }
    }

    sscore *= 2.0 / (2.0 + T);

    /*if (sscore > 0)
    {
        Simulator tsim = ssim;
        tsim.robots.clear();

        int minEnemyT = 100;
        int minMyT = 100;
        for (const MyRobot &r : ssim.robots)
        {

            EngagePoint engagePoint = computeEngagePoint(tsim, r, std::min(15, (int) (T*TICKS_PER_SECOND)));
            if (engagePoint.found)
            {
                if (r.side == Side::POS)
                {
                    if (minEnemyT > engagePoint.targetT)
                        minEnemyT = engagePoint.targetT;
                }
                else
                {
                    if (minMyT > engagePoint.targetT)
                        minMyT = engagePoint.targetT;
                }
            }

            if (minEnemyT > 5 && minEnemyT < 100 && minEnemyT < minMyT)
            {
                sscore = sscore * 0.5 - 500;
                //LOG("SSS " << sscore);
            }
            else if (minMyT > 15 && minMyT < 100)
            {
                sscore += 50;
            }
        }
    }*/

    if (sscore > 0)
    {
        P dir = ball.vel.xz().norm();
        F spd = ball.vel.xz().len();

        for (const MyRobot &r : robots)
        {
            if (r.side == Side::POS)
            {
                P pp = r.body.pos.xz() + r.body.vel.xz() * 0.2;
                P pd = pp - ball.pos.xz();
                F d = dot(pd, dir);
                if (d > 5)
                {
                    F pt = d / spd;
                    F y = ball.pos.y + ball.vel.y * pt - GRAVITY * 0.5 * sqr(pt);

                    F Y = 4;
                    if (r.nitro > 15)
                        Y = 6;

                    if (y < Y)
                    {
                        P bp = ball.pos.xz() + ball.vel.xz() * pt;
                        F l = pp.dist(bp);

                        if (l < (pt * 30 + 3) && l < 15)
                        {
                            sscore -= 1000/(l + 3);
                        }
                    }
                }
            }
        }
    }

    F vb = ball.vel.len();
    if (vb < 10)
    {
        sscore -= sqr(10 - vb);
    }

    return sscore;
}

F ballScore(const Simulator &ssim, const std::vector<MyRobot> &robots)
{
    const Ball &ball = ssim.ball;
    F sscore = 0;

    Simulator testSim = ssim;
    testSim.robots.clear();

    std::vector<P3> renderPoints;

    F coef = 1.0;
    for (size_t t = 0; t < 130; ++t)
    {
        renderPoints.push_back(testSim.ball.pos);

        testSim.tickFastUltra();
        if (testSim.scoreSide != Side::NONE)
        {
            F goalScore = 1000 + (testSim.ball.pos.y - BALL_RADIUS) * 100;// + testSim.ball.vel.len() * 10;

            if (testSim.scoreSide == Side::NEG)
            {
                sscore += goalScore * coef;
            }
            else
            {
                sscore -= goalScore * coef;
            }
            break;
        }

        coef *= 0.99;

        if (t == 30)
            sscore += testSim.ball.vel.z;

        /*if (testSim.ball.pos.xz().dist2(P(0, -ARENA_DEPTH * 0.5)) < sqr(ARENA_GOAL_WIDTH * 0.5))
        {
            sscore -= 5 * coef;
        }
        else if (testSim.ball.pos.xz().dist2(P(0, ARENA_DEPTH * 0.5)) < sqr(ARENA_GOAL_WIDTH * 0.5))
        {
            sscore += 5 * coef;
        }*/
    }

    return sscore + ballScoreFast(ssim, robots) * 0.5;
}

constexpr P POS_GOAL_POS = P(0, ARENA_DEPTH * 0.5 + BALL_RADIUS);
constexpr P NEG_GOAL_POS = P(0, -(ARENA_DEPTH * 0.5 + BALL_RADIUS));

constexpr P POS_GOAL_POS_NEAR = P(0, ARENA_DEPTH * 0.5 + 10);
constexpr P NEG_GOAL_POS_NEAR = P(0, -(ARENA_DEPTH * 0.5 + 10));

P3 getTargetPos(const Ball &ball, const MyRobot &me)
{
    P bpos = ball.pos.xz();
    P bvel = ball.vel.xz();
    Side mySide = me.side;

    // 10 - 460
    // 11 - 462
    // 12 - 438
    // 13 - 290
    // 14 - 189
    // 15 - 137/514
    // 16 - 107
    // 17 - 101
    // 18 - 121
    // 19 -148
    // 20 - 177/514

    // h 6.5 - 50
    // h 6 - 28
    // h 5.5 - 25

    if (bpos.z < (-ARENA_DEPTH * 0.5) && ball.pos.y > 6.5 && mySide == Side::NEG)
    {
        F dz = (-ARENA_DEPTH * 0.5) - bpos.z;
        return P3(bpos.x, ROBOT_RADIUS, -ARENA_DEPTH * 0.5 - 18 + dz*2);
    }

    F l = bvel.len();
    if (l > 0)
    {
        if (l > 50)
            bvel /= l;
        else
            bvel /= (50);
    }

    bool nearGoal = std::abs(bpos.z) > (ARENA_DEPTH * 0.5 - 5) && std::abs(bpos.x) < (ARENA_GOAL_WIDTH * 0.5);

    P targetPos = bpos + ((bpos - (mySide == Side::NEG ? (nearGoal ? POS_GOAL_POS_NEAR : POS_GOAL_POS) : (nearGoal ? NEG_GOAL_POS_NEAR : NEG_GOAL_POS))).norm() + bvel ).norm() * ((ROBOT_RADIUS + BALL_RADIUS) * 0.65f);

    return xz(targetPos, ROBOT_RADIUS);
}

F optimizeKick(const Simulator &sim, MyRobot &me, int enEngageT);

void updateActions(double jump_speed, Simulator &tsim, int t, int meId, const std::map<int, RobotState> &robotState, bool useNitro)
{
    for (MyRobot &r : tsim.robots)
    {
        const RobotState &enSt = robotState.find(r.id)->second;

        r.action.target_velocity = xz((enSt.foundTargetPos - r.body.pos).xz().norm(), (useNitro ? ROBOT_MAX_JUMP_SPEED : 0)) * enSt.speed;
        r.action.use_nitro = useNitro;

        if (r.id == meId)
        {
            r.action.jump_speed = jump_speed;
        }
        else
        {
            F estT = std::sqrt(2 * (enSt.height - ROBOT_RADIUS) / GRAVITY);
            if (estT + t > enSt.targetT)
            {
                r.action.jump_speed = std::sqrt(2 * (enSt.height - ROBOT_RADIUS) * GRAVITY);
            }

            if (t + 5 > enSt.targetT && !r.touch || r.side == Side::POS && enSt.jumpT >= 0 && enSt.jumpT <= t)
            {
                r.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
            }
        }
    }
};

void MyStrat::computeJump(MyRobot &me)
{
    RobotState &st = robotState[me.id];

    F enemyBallDist = 1e6;
    for (const MyRobot &en : sim.robots)
    {
        if (en.side == Side::POS)
            enemyBallDist = std::min(enemyBallDist, length(en.body.pos - sim.ball.pos) - ROBOT_RADIUS - BALL_RADIUS);
    }

    F currentScore = ballScore(sim, sim.robots);

    constexpr int DEPTH_T = 30;

    if (st.targetT >= DEPTH_T)
        return;

    bool compute = me.touch;
    if (!compute)
    {
        CollisionInfo info = dan_to_arena(me.body.pos);
        if (info.distance < 1.5)
            compute = true;
    }

    if (compute)
    {
        F initEstJumpSpeed = clamp(st.height > ROBOT_RADIUS ? std::sqrt((st.height - ROBOT_RADIUS) * 2 * GRAVITY) : 0, 0, ROBOT_MAX_JUMP_SPEED);
        int nitroTicks = 0;

        //if (currentScore < -200 || sim.ball.pos.z > 30)
        {
            if (highDanger && st.height > 9 && me.nitro > 20)
            {
                nitroTicks = 22;
                initEstJumpSpeed = ROBOT_MAX_JUMP_SPEED;
            }
            else if (st.height > 8 && me.nitro > 14)
            {
                nitroTicks = 14;
                initEstJumpSpeed = ROBOT_MAX_JUMP_SPEED;
            }
            else if (st.height > 6 && me.nitro > 7)
            {
                nitroTicks = 8;
                initEstJumpSpeed = ROBOT_MAX_JUMP_SPEED;
            }
        }

        F score = -1e6;
        int jumpTime = -1;
        F jumpSpeed = initEstJumpSpeed;

        //for (int k = 0; k < 1; ++k)
        {
            F estJumpSpeed = initEstJumpSpeed;
            /*if (k == 1)
            {
                if (estJumpSpeed > ROBOT_MAX_JUMP_SPEED - 3)
                    continue;

                estJumpSpeed = ROBOT_MAX_JUMP_SPEED;
            }*/
            //F estJumpSpeed = (1.0 + 0.15 * k) * initEstJumpSpeed;


            Simulator testSim = sim;

            auto updateScore = [&score, &jumpTime, &jumpSpeed, this](const Simulator &ssim, int jmpT, F jumpS){
                F sscore = ballScore(ssim, sim.robots);

                sscore /= (5 + jmpT);
                //if (jmpT > enEngageT - 5)
                //    sscore -= (jmpT - enEngageT + 5) * 50;

                if (score <= sscore)
                {
                    score = sscore;
                    jumpTime = jmpT;
                    jumpSpeed = jumpS;
                }
            };

            for (int i = 0; i < DEPTH_T; ++i)
            {
                if (i % 2 == 0)
                {
                    Simulator jsim = testSim;
                    bool ballTouch = false;
                    for (int ji = i; ji < DEPTH_T; ++ji)
                    {
                        int rind = jsim.indexOfRobot(me.id);

                        int di = ji - i;
                        if (ji <= i + 1)
                            updateActions(estJumpSpeed, jsim, ji, me.id, robotState, di < nitroTicks);
                        else
                            updateActions(ROBOT_MAX_JUMP_SPEED, jsim, ji, me.id, robotState, di < nitroTicks);

                        if (ji == 0)
                        {
                            jsim.tick();
                        }
                        else
                        {
                            jsim.tickFast();
                        }

                        ballTouch |= jsim.robots[rind].ballTouch;
                    }

                    if (i == 0)
                        renderSphere(jsim.robots[jsim.indexOfRobot(me.id)].body.pos, 0.03, 0x00ff0000);
                    if (ballTouch)
                        updateScore(jsim, i, estJumpSpeed);
                }

                if (jumpTime > 0)
                    break;

                updateActions(0, testSim, i, me.id, robotState, false);
                //updateActions(0, testSim, i);
                testSim.tickFast();
            }

            if (jumpTime <= 0)
            {
                //renderSphere(testSim.robots[testSim.indexOfRobot(me.id)].body.pos, 0.03, 0x00ffff00);
                updateScore(testSim, DEPTH_T - 1, estJumpSpeed);
            }

            //if (jumpTime >= 0)
            //   LOG("JUMPT " << jumpTime);

            if (jumpTime == 0)
            {


                if (enemyBallDist < 10 || score > 100 || score > currentScore + 500 || sim.ball.pos.z < -5)
                {
                    me.action.jump_speed = jumpSpeed;
                    st.nitroTicks = nitroTicks;
                    me.action.target_velocity = xz((st.foundTargetPos - me.body.pos).xz().norm(), (nitroTicks ? ROBOT_MAX_JUMP_SPEED : 0)) * st.speed;
                    st.targetVel = me.action.target_velocity;

                    WRITE_LOG("JUMP " << jumpSpeed);
                    //LOG("JUMP " << (jumpSpeed) / initEstJumpSpeed);

                    if (nitroTicks == 0)
                    {
                        F bestScore = -1e6;

                        for (int k = -2; k <= 2; ++k)
                        {
                            F newJumpSpeed = (1.0 + 0.1 * k) * jumpSpeed;
                            if (newJumpSpeed > ROBOT_MAX_JUMP_SPEED)
                                break;


                            Simulator testSim = sim;
                            for (int i = 0; i < DEPTH_T; ++i)
                            {
                                if (i <= 1)
                                    updateActions(newJumpSpeed, testSim, i, me.id, robotState, false);
                                    //updateActions(estJumpSpeed, jsim, ji);
                                else
                                    updateActions(ROBOT_MAX_JUMP_SPEED, testSim, i, me.id, robotState, false);

                                if (i == 0)
                                    testSim.tick();
                                else
                                    testSim.tickFast();

                                if (testSim.robots[testSim.indexOfRobot(me.id)].ballTouch)
                                {
                                    F score = ballScore(testSim, sim.robots);

                                    if (score > bestScore)
                                    {
                                        bestScore = score;
                                        me.action.jump_speed = newJumpSpeed;
                                    }

                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

F optimizeKick(const Simulator &sim, MyRobot &me, int enEngageT)
{
    F bestScore = -1e6;

    if ((sim.ball.pos - me.body.pos).len() < (BALL_RADIUS + ROBOT_MAX_RADIUS + 1))
    {
        Simulator testSim = sim;
        int ind = testSim.indexOfRobot(me.id);

        testSim.robots[ind].action.jump_speed = ROBOT_MAX_JUMP_SPEED;
        testSim.tickFast();

        if (testSim.robots[ind].ballTouch)
        {

            for (int i = 0; i <= 10; ++i)
            {
                F jumpSpeed = i / 10.0 * ROBOT_MAX_JUMP_SPEED;

                F scoreSum = 0;

                for (int k = 0; k < 5; ++k)
                {
                    setFixedEInd(k);
                    Simulator testSim2 = sim;

                    testSim2.robots[ind].action.jump_speed = jumpSpeed;
                    testSim2.tick();

                    F score = ballScore(testSim2, sim.robots);
                    scoreSum += score;
                }
                resetFixedEInd();

                if (scoreSum > bestScore)
                {
                    bestScore = scoreSum;
                    me.action.jump_speed = jumpSpeed;
                }
            }
        }
    }

    return bestScore;
}

void MyStrat::computePredictedBallPos()
{
}

F computeTrajScore(const Simulator &sim, int meId, const Trajectory2d &traj, int enEngageT)
{
    Simulator tsim = sim;
    P mePos = tsim.robots[tsim.indexOfRobot(meId)].body.pos.xz();

    filterVector(tsim.robots, [meId, &mePos, &sim](const MyRobot &r){return r.id != meId && (r.touch || r.body.pos.xz().dist(mePos) > 20 && r.body.pos.xz().dist(sim.ball.pos.xz()) > 20);});
    int meInd = tsim.indexOfRobot(meId);

    std::vector<P3> renderPoints;
    renderPoints.push_back(tsim.robots[meInd].body.pos);

    bool ballTouch = false;

    for (const P &p : traj.vel)
    {
        tsim.robots[meInd].action.target_velocity = xz(p, 0);
        tsim.robots[meInd].action.jump_speed = 0;
        tsim.robots[meInd].action.use_nitro = false;
        tsim.tick();

        //if (tsim.robots[meInd].body.pos.z > tsim.ball.pos.z)
        ballTouch |= tsim.robots[meInd].ballTouch;

        renderPoints.push_back(tsim.robots[meInd].body.pos);
    }

    F score = 0;
    if (ballTouch)
        score -= 1000;

    ballTouch = false;

    tsim.robots[meInd].action.jump_speed = ROBOT_MAX_JUMP_SPEED;
    tsim.robots[meInd].action.target_velocity = xz(traj.lastVel, traj.nitroTicks > 0 ? ROBOT_MAX_JUMP_SPEED : 0);
    tsim.robots[meInd].action.use_nitro = traj.nitroTicks > 0;
    tsim.tick();
    ballTouch |= tsim.robots[meInd].ballTouch;

    for (int i = 1; i < traj.flyT; ++i)
    {
        tsim.robots[meInd].action.jump_speed = 0;
        tsim.robots[meInd].action.use_nitro = i < traj.nitroTicks;
        tsim.tick();
        ballTouch |= tsim.robots[meInd].ballTouch;

        renderPoints.push_back(tsim.robots[meInd].body.pos);
    }

    tsim.robots[meInd].action.jump_speed = ROBOT_MAX_JUMP_SPEED;

    for (int i = 0; i < 5; ++i)
    {
        setFixedEInd(i);
        Simulator tsim2 = tsim;
        tsim2.tick();
        ballTouch |= tsim2.robots[meInd].ballTouch;

        renderPoints.push_back(tsim2.robots[meInd].body.pos);

        if (!ballTouch)
        {
            resetFixedEInd();
            return -1e6;
        }

        score += ballScore(tsim2, sim.robots);

        F relSpd = (tsim2.robots[meInd].body.vel - tsim2.ball.vel).xz().len();
        if (relSpd > 35)
        {
            score -= sqr(relSpd - 35);
            //LOG("S " << relSpd);
        }

        F vb = tsim2.ball.vel.len();
        if (vb < 30)
        {
            score -= sqr(30 - vb);
        }
    }
    resetFixedEInd();

    renderLineV(renderPoints, 0x44ff4400);

    int trajLen = (int) traj.vel.size() + traj.flyT;
    if (trajLen >= enEngageT)
        score -= 500;

    if (trajLen > enEngageT - 15)
    {
        score -= 50;
        score -= (trajLen - enEngageT + 15) * 10;
    }

    score -= (traj.vel.size() + traj.flyT) * 30.0;
    return score;
}

F computeFlyScore(const Simulator &sim, int meId, int enEngageT, P3 targetVel, int nitroTicks)
{
    Simulator tsim = sim;
    P mePos = tsim.robots[tsim.indexOfRobot(meId)].body.pos.xz();
    filterVector(tsim.robots, [meId, &mePos](const MyRobot &r){return r.id != meId && (r.touch || r.body.pos.xz().dist(mePos) > 20);});
    tsim.nitro_packs.clear();

    for (MyRobot &r : tsim.robots)
    {
        r.action.target_velocity = r.body.vel;
        r.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
    }

    int ind = tsim.indexOfRobot(meId);
    bool ballTouch = false;
    Simulator tsimSave;

    F score = 0;
    for (int t = 0; t < 30; ++t)
    {
        tsim.robots[ind].action.target_velocity = targetVel;
        tsim.robots[ind].action.jump_speed = 0;
        tsim.robots[ind].action.use_nitro = t < nitroTicks;

        tsimSave = tsim;
        tsim.tickFast();
        ballTouch |= tsim.robots[ind].ballTouch;

        if (ballTouch)
        {
            tsim = tsimSave;
            tsim.robots[ind].action.jump_speed = ROBOT_MAX_JUMP_SPEED;

            for (int i = 0; i < 5; ++i)
            {
                setFixedEInd(i);
                Simulator tsim2 = tsim;
                tsim2.tick();

                score += ballScore(tsim2, sim.robots);

                F relSpd = (tsim2.robots[ind].body.vel - tsim2.ball.vel).xz().len();
                if (relSpd > 35)
                {
                    score -= sqr(relSpd - 35);
                }

                F vb = tsim2.ball.vel.len();
                if (vb < 30)
                {
                    score -= sqr(30 - vb);
                }
            }
            resetFixedEInd();

            if (t >= enEngageT)
                score -= 500;

            if (t > enEngageT - 15)
            {
                score -= 50;
                score -= (t - enEngageT + 15) * 10;
            }

            return score;
        }
    }

    return -1e6;
}

struct KickParams
{
    MyRobot found;
    int T = 0;
    int nitroTicks = 0;
};

KickParams findKickParams(const Simulator &sim, const MyRobot &me, const Ball &b, const P&vel, int ct, bool highDanger)
{
    KickParams result;

    Simulator tsim = sim;
    tsim.robots.clear();
    tsim.robots.push_back(me);

    F bestScore = -1e6;

    P dir = (b.pos.xz() - P(0, ARENA_DEPTH * 0.5 + BALL_RADIUS)).norm();

    size_t sz = highDanger ? JUMP_CONSTANTS.nitroTicks.size() : JUMP_CONSTANTS.nitroTicks.size() - 1;
    F nitroPoints = NITRO_POINTS;
    if (highDanger)
        nitroPoints *= 0.5;

    for (size_t nitroTp = 0; nitroTp < sz; ++nitroTp)
    {
        F consumption = JUMP_CONSTANTS.nitroConsumption[nitroTp];
        if (me.nitro < consumption /*|| nitroTp > 0 && b.pos.y < 6*/)
            break;

        for (int k = -5; k <= 5; k += 1)
        {
            for (int t = 0; t < JUMP_CONSTANTS.jumpAttrs[nitroTp].size() && ct >= t; ++t)
            {
                const JumpAttrs &ja = JUMP_CONSTANTS.jumpAttrs[nitroTp][t];
                tsim.ball = b;
                constexpr F R2 = sqr(BALL_RADIUS + ROBOT_RADIUS * 1.03);
                F dh2 = sqr(tsim.ball.pos.y - ja.h);
                if (R2 > dh2)
                {
                    F d = std::sqrt(R2 - dh2);
                    tsim.robots[0].body.pos = xz(b.pos.xz() + dir.rotate(P(k/10.0)) * d, ja.h);
                    tsim.robots[0].body.vel = xz(vel, ja.v);
                    tsim.robots[0].action.target_velocity = P3(0);
                    tsim.robots[0].action.jump_speed = ROBOT_MAX_JUMP_SPEED;

                    MyRobot rr = tsim.robots[0];

                    tsim.microtick();

                    // F score = ballScore(tsim);

                    F score = ballScoreFast(tsim, sim.robots) - nitroTp * nitroPoints;
                    //LOG("S " << score << " " << tsim.robots[0].ballTouch);

                    //WRITE_LOG("K " << k << " " << t);
                    //WRITE_LOG("id " << me.id << " K " << k << " t " << t << " SC " << score);

                    if (score > bestScore)
                    {
                        bestScore = score;
                        result.T = t;
                        result.found = rr;
                        result.nitroTicks = JUMP_CONSTANTS.nitroTicks[nitroTp];
                        //LOG("BS " << bestScore);
                    }
                }
            }
        }
    }

    return result;
}

F MyStrat::computePP(const P &pos, int myId, int T)
{
    F t = T * TICK_DUR;
    RobotState &st = robotState[myId];

    P tp = st.foundTargetPos.xz();
    F res = 1000/(1 + tp.dist2(pos));

    P3 bp = sim.ball.pos;

    MyRobot &me = sim.robots[sim.indexOfRobot(myId)];
    //F t = std::max(0.0, bp.xz().dist(me.body.pos.xz()) - 3) / 50.0 + T;

    bp += sim.ball.vel * t;
    bp.y -= GRAVITY * sqr(t) * 0.5;

    if (bp.y < 4)
    {
        //P mgDir = ((bp.xz() - P(0, -ARENA_DEPTH * 0.5 - 5)).norm() + (P(0, ARENA_DEPTH * 0.5 + 5) - bp.xz()).norm()).norm();
        P mgDir = P(0, 1);
        F l2 = ((bp.xz() + mgDir * 3 - pos) * P(1, 0.7)).len2();
        if (l2 < sqr(5))
        {
            res -= 2000 / (1 + l2);
        }
    }
    RobotState &mySt = robotState[myId];

    SideState &ss = sideState[(int) me.side];


        for (const MyRobot &r : sim.robots)
        {
            if (r.side == Side::NEG && r.id != myId)
            {
                if (r.body.pos.xz().dist2(me.body.pos.xz()) < sqr(10));
                {
                    RobotState &st = robotState[r.id];

                    //t = std::max(0.0, r.body.pos.xz().dist(me.body.pos.xz()) - 2) / 50.0 + T;
                    P rp = r.body.pos.xz() + r.body.vel.xz() * t;

                    F l2 = rp.dist2(pos);
                    if (st.attackScore > mySt.attackScore && l2 < sqr(3))
                    {
                        res -= 1000 / (1 + l2);
                    }

                    //if (myId != ss.attackId && me.id != ss.defendId)
                    //    res -= 10 / (5 + l2);
                }
            }
            /*else if (r.side == Side::POS)
            {
                if (r.body.pos.xz().dist2(me.body.pos.xz()) < sqr(10));
                {
                    P dir = (sim.ball.pos.xz() - r.body.pos.xz()).norm();
                    P tp = r.body.pos.xz() + dir;
                    F l2 = tp.dist2(pos);
                    res += 10 / (1 + l2);
                }
            }*/
        }


    if (me.nitro < 100)
    {
        for (const MyNitroPack &p : sim.nitro_packs)
        {
            if (p.respawn_ticks <= T)
            {
                F l2 = p.pos.xz().dist2(pos);
                if (p.pos.z > 0)
                {
                    if (l2 < sqr(15))
                    {
                        res += 1.0 / (1.0 + l2);
                    }
                }
                else if (me.nitro < 30 && sim.ball.vel.z > 10 && l2 < sqr(15) || l2 < sqr(5))
                {
                    res += 1.0 / (1.0 + l2);
                }

                if (l2 < sqr(ROBOT_RADIUS + NITRO_PACK_RADIUS))
                {
                    res += (100 - me.nitro) * 10 * (st.targetT < 20 ? 0.1 : 1.0);
                }
            }
        }
    }

    return res;
}

void MyStrat::computeHighDanger()
{
    highDanger = false;
    if (currentBallScore < -500)
    {
        highDanger = true;
        for (const MyRobot &me : sim.robots)
        {
            if (me.side == Side::NEG)
            {
                RobotState &st = robotState[me.id];
                if (st.traj.found || st.targetPointFound)
                {
                    highDanger = false;
                    break;
                }
            }
        }
    }
}

void MyStrat::updateRobotState()
{
    std::map<int, EngagePoint> engagePoints;

    enEngageT = 10000;
    for (MyRobot &me : sim.robots)
    {
        if (me.side == Side::POS)
        {
            engagePoints.insert(std::make_pair(me.id, computeEngagePoint(sim, me, PRED_COUNT, highDanger, robotState)));
            EngagePoint &engagePoint = engagePoints.find(me.id)->second;
            if (engagePoint.found && enEngageT > engagePoint.targetT)
            {
                enEngageT = engagePoint.targetT;
            }

            if (!engagePoint.found)
            {
                int T = (int)(1.3 * (me.body.pos - sim.ball.pos).len() * TICKS_PER_SECOND / MAX_ENTITY_SPEED);

                if (enEngageT > T)
                {
                    enEngageT = T;
                }
            }

            RobotState &st = robotState[me.id];
            st.jumpT = -1;

            F bestEnScore = currentBallScore - 300;

            F lastScore = 0;
            if (me.touch && engagePoint.found)
            {
                Simulator tsim = sim;
                filterVector(tsim.robots, [&me](const MyRobot &r){return r.id != me.id;});
                tsim.nitro_packs.clear();
                tsim.robots[0].action.use_nitro = false;
                tsim.robots[0].action.jump_speed = 0;
                tsim.robots[0].action.target_velocity = tsim.robots[0].body.vel;

                for (int t = 0; t < 15; ++t)
                {
                    bool ballTouch = false;
                    Simulator tsim2 = tsim;
                    for (int j = t; j < 15; ++j)
                    {
                        tsim2.robots[0].action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                        if (j == t)
                            tsim2.tickFast();
                        else
                            tsim2.tickFastUltra();
                        ballTouch = tsim2.robots[0].ballTouch;
                    }

                    if (ballTouch)
                    {
                        F ballScore = ballScoreFast(tsim2, sim.robots);
                        if (ballScore < bestEnScore)
                        {
                            bestEnScore = ballScore;
                            st.jumpT = t;
                        }
                    }

                    tsim.tickFast();
                }

                lastScore = ballScoreFast(tsim, sim.robots);
                if (lastScore <= bestEnScore)
                    st.jumpT = -1;
            }

            if (st.jumpT >= 0)
            {
                WRITE_LOG("J " << st.jumpT << " " << bestEnScore << " " << currentBallScore);
            }
        }
    }

    for (MyRobot &me : sim.robots)
    {
        if (me.side == Side::NEG)
        {
            engagePoints.insert(std::make_pair(me.id, computeEngagePoint(sim, me, PRED_COUNT, highDanger, robotState)));

            EngagePoint &engagePoint = engagePoints.find(me.id)->second;
            SideState &ss = sideState[(int) me.side];
            if (ss.defendId == me.id)
            {
                if (engagePoint.found && engagePoint.targetT > 70 && enEngageT < 50)
                {
                    engagePoint.foundTargetPos = P3(0, ROBOT_RADIUS, -sim.arena->depth * 0.5);
                    engagePoint.targetT = 0;
                    engagePoint.height = ROBOT_RADIUS;
                    engagePoint.found = false;
                }
            }

            RobotState &st = robotState[me.id];
            st.targetPointFound = engagePoint.found;
        }
    }

    for (MyRobot &me : sim.robots)
    {
        RobotState &st = robotState[me.id];
        SideState &ss = sideState[(int) me.side];
        F sideSign = getSideSign(me.side);

        EngagePoint &engagePoint = engagePoints.find(me.id)->second;

        st.trajScore = -1e6;
        if (st.traj.found && !st.traj.vel.empty())
        {
            st.traj.vel.erase(st.traj.vel.begin());
            st.trajScore = computeTrajScore(sim, me.id, st.traj, enEngageT);
            if (st.trajScore < -1000)
                st.traj.found = false;
        }
        else
        {
            st.traj.found = false;
        }

        st.speed = ROBOT_MAX_GROUND_SPEED;
        if (engagePoint.found && engagePoint.targetT < 60 && me.side == Side::NEG && me.touch)
        {
            for (int ct = engagePoint.targetT; ct < engagePoint.targetT + 6; ++ct)
            {
                const Ball &b = engagePoint.predictedBallIterator.sim.ball;

                bool trajFound = false;
                for (int iter = 0; iter < 2; ++iter)
                {
                    P spd = engagePoint.arrivalSpd;
                    if (iter == 0)
                    {
                        spd = engagePoint.arrivalSpd;
                    }
                    else if (iter == 1)
                    {
                        //if (engagePoint.arrivalSpd.z < 25 && engagePoint.arrivalSpd.z > 15)
                        //    continue;

                        spd = P(0, 20);
                    }
                    /*else if (iter == 2)
                    {
                        if (engagePoint.arrivalSpd.z > 10)
                            continue;

                        spd = P(0, 10);
                    }*/

                    KickParams kickParams = findKickParams(sim, me, b, spd, ct, highDanger);

                    Trajectory2d traj;
                    traj.robot2d = Robot2d(me);

                    bool res = traj.find((kickParams.found.body.pos - kickParams.found.body.vel * kickParams.T * TICK_DUR).xz(), kickParams.found.body.vel.xz(), ct - kickParams.T);
                    WRITE_LOG("id " << me.id << " R " << res);

                    if (res)
                    {
                        traj.flyT = kickParams.T;
                        traj.nitroTicks = kickParams.nitroTicks;
                        if (!traj.vel.empty())
                            traj.lastVel = *traj.vel.rbegin();
                        else
                            traj.lastVel = P(0, 0);

                        //F ss = computeTrajScore(sim, me.id, traj);
                        //LOG("SS " << ss);

                        if (st.traj.found)
                        {
                            F newScore = computeTrajScore(sim, me.id, traj, enEngageT);

                            if (newScore < -1000)
                            {
                                WRITE_LOG("ERR " << newScore);
                                //newScore = computeTrajScore(sim, me.id, traj);
                            }
                            else
                            {
                                WRITE_LOG("SCORE " << newScore << " P " << kickParams.found.body.pos);
                            }

                            if (newScore > st.trajScore)
                            {
                                st.traj = traj;
                                st.trajScore = newScore;
                            }
                        }
                        else
                        {
                            st.traj = traj;
                            st.trajScore = computeTrajScore(sim, me.id, traj, enEngageT);
                        }

                        /*if (st.trajScore < -1000)
                            LOG("SS " << st.trajScore);*/
                        if (st.trajScore < -10000)
                        {
                            st.traj.found = false;
                        }
                        else
                        {
                            trajFound = true;
                        }
                    }
                }

                if (trajFound)
                    break;

                if (!engagePoint.predictedBallIterator.computeTick(robotState))
                    break;
            }
        }

        if (st.traj.found)
        {
            WRITE_LOG("SCORE " << me.id << " " << st.trajScore << " T " << st.traj.vel.size());
        }

        if (!st.traj.found)
        {
            F speed = ROBOT_MAX_GROUND_SPEED;
            bool guardGoal = false;

            if (ss.defendId == me.id)
            {
                if (engagePoint.foundTargetPos.z * sideSign > -3 * sideSign)
                {
                    guardGoal = true;
                }
                else
                {
                    F goalLen = (P3(0, ROBOT_RADIUS, -sim.arena->depth * 0.5 * sideSign) - engagePoint.foundTargetPos).len();

                    if (goalLen > (sim.arena->goal_width))
                    {
                        guardGoal = true;
                    }
                }
            }

            if (guardGoal)
            {
                engagePoint.foundTargetPos = P3(0, ROBOT_RADIUS, -sim.arena->depth * 0.5 * sideSign);
                engagePoint.height = ROBOT_RADIUS;
            }
            else if (engagePoint.arrivalT > 0 && engagePoint.found)
            {

                F targetSpeed = ROBOT_MAX_GROUND_SPEED * engagePoint.arrivalT / engagePoint.targetT;

                F bestDist = 1e8;

                for (int k = 0; k <= 10; ++k)
                {
                    F spd = (k * 0.04 + 0.8) * targetSpeed;
                    if (spd > ROBOT_MAX_GROUND_SPEED)
                        break;

                    Robot2d robot2d = Robot2d(me);

                    for (size_t t = 0; t < engagePoint.targetT; ++t)
                    {
                        robot2d.tick((engagePoint.foundTargetPos.xz() - robot2d.pos).norm() * spd);
                    }

                    F d = (robot2d.pos - engagePoint.foundTargetPos.xz()).len();


                    /*P3 vel = me.body.vel;
                    P3 pos = me.body.pos;
                    for (size_t t = 0; t < targetT; ++t)
                    {
                        P3 targetVel = (foundTargetPos - pos).norm() * spd;
                        P3 target_velocity_change = targetVel - vel;

                        vel += clampV(
                                (target_velocity_change).norm() * ROBOT_ACCELERATION * TICK_DUR,
                                length(target_velocity_change));

                        pos += vel * TICK_DUR;
                    }

                    F d = (pos - foundTargetPos).len();*/



                    if ( d < bestDist )
                    {
                        bestDist = d;
                        speed = spd;
                    }
                }

                //LOG("SP " << speed/targetSpeed);
            }

            st.speed = speed;

            renderSphere(engagePoint.foundTargetPos, 0.1, 0xff000000);
            renderLine(me.body.pos, engagePoint.foundTargetPos, 0xff000000);
        }

        st.foundTargetPos = engagePoint.foundTargetPos;
        st.height = engagePoint.height;
        st.targetT = engagePoint.targetT;
    }

    for (MyRobot &me : sim.robots)
    {
        if (me.side != Side::NEG)
            continue; // TODO

        SideState &ss = sideState[(int) me.side];
        F sideSign = getSideSign(me.side);

        bool guardGoal = false;
        RobotState &st = robotState[me.id];
        P3 targetPos = st.foundTargetPos;

        if (ss.defendId == me.id && st.foundTargetPos.z * sideSign> -sim.arena->depth * 0.45 * sideSign)
        {
            for (MyRobot &oth : sim.robots)
            {
                if (oth.side != me.side || oth.id == me.id)
                    continue;

                P3 othTargetPos = robotState[oth.id].foundTargetPos;

                if ((targetPos - othTargetPos).len() < ROBOT_RADIUS * 4)
                {
                    if (robotState[oth.id].targetT <= st.targetT)
                    {
                        guardGoal = true;
                        break;
                    }
                }
            }
        }

        if (guardGoal)
        {
            st.foundTargetPos = P3(0, ROBOT_RADIUS, -sim.arena->depth * 0.5 * sideSign);
            st.height = ROBOT_RADIUS;
            st.traj.found = false; // TODO optimize
            //LOG("G " << me.id);
        }
    }

    F bestTrajScore = -1e6;
    int bestTrajScoreId = -1;
    for (MyRobot &me : sim.robots)
    {
        if (me.side != Side::NEG)
            continue;

        RobotState &st = robotState[me.id];
        if (st.traj.found)
        {
            if (st.trajScore > bestTrajScore)
            {
                bestTrajScore = st.trajScore;
                bestTrajScoreId = me.id;
            }
        }
    }

    if (!sim.nitro_packs.empty())
    {
        F bestGotoNitroScore = -1e6;
        int gotoNitroId = -1;
        std::map<int, P> pt;
        for (MyRobot &me : sim.robots)
        {
            if (me.side != Side::NEG || !me.touch || me.nitro > 20)
                continue;

            RobotState &st = robotState[me.id];
            if (st.traj.found)
                continue;

            SideState &ss = sideState[(int) Side::POS];
            if (st.targetT > 60 || ss.defendId == me.id && st.foundTargetPos == P3(0, ROBOT_RADIUS, -ARENA_DEPTH * 0.5))
            {
                F score = (20 - me.nitro)*5;
                F minL = 1000;
                for (const MyNitroPack&p : sim.nitro_packs)
                {
                    F l = me.body.pos.xz().dist(p.pos.xz());
                    if (p.respawn_ticks < 1.9 * l && l < minL)
                    {
                        minL = l;
                        pt[me.id] = p.pos.xz();
                    }
                }

                if (minL < 25)
                {
                    score -= minL;
                    if (score > bestGotoNitroScore)
                    {
                        bestGotoNitroScore = score;
                        gotoNitroId = me.id;
                    }
                }
            }
        }

        if (gotoNitroId >= 0)
        {
            RobotState &st = robotState[gotoNitroId];
            st.foundTargetPos = xz(pt[gotoNitroId], ROBOT_RADIUS);
            st.speed = ROBOT_MAX_GROUND_SPEED;

            //LOG("T " << gotoNitroId << " " << st.foundTargetPos << " " << st.targetT);
        }
    }

    if (bestTrajScoreId != -1)
    {
        SideState &ss = sideState[(int) Side::NEG];
        ss.attackId = bestTrajScoreId;
    }

    // PPF
    for (MyRobot &me : sim.robots)
    {
        if (me.side != Side::NEG || !me.touch)
            continue;

        RobotState &st = robotState[me.id];

        if (st.traj.found && bestTrajScoreId != -1 && me.id != bestTrajScoreId && st.trajScore < bestTrajScore - 500)
        {
            st.traj.found = false;
            //LOG("BS " << bestTrajScore << " " << st.trajScore);
        }

        if (!st.traj.found)
        {
            //LOG("NF " << me.id);

            auto compScore = [this](const P &p, const MyRobot &me){
                F s = 0;
                Robot2d robot2d = Robot2d(me);
                F expF = 1.0;
                for (int t = 0; t <= 15; ++t)
                {
                    //P tv = (p - robot2d.pos).norm() * st.speed;

                    robot2d.tick((p - robot2d.pos) / TICK_DUR);
                    //if (t % 4 == 3)
                    {
                        s += computePP(robot2d.pos, me.id, t) * expF;
                        expF *= 0.95;
                        //renderSphere(xz(robot2d.pos, 1), 0.1, 0x0000ff00);
                    }
                }

                return s;
            };

            F bestScore = compScore(st.foundTargetPos.xz(), me);
            P3 newTargetPos = st.foundTargetPos;

            constexpr F CNT = 20;
            for (F i = 0; i < CNT; ++i)
            {
                P dir = P(2 * M_PI * i / CNT);
                P p = me.body.pos.xz() + dir * 10;

                F s = compScore(p, me);
                if (s > bestScore)
                {
                    bestScore = s;
                    newTargetPos = xz(p, ROBOT_RADIUS);
                }
            }

            st.foundTargetPos = newTargetPos;
            renderSphere(st.foundTargetPos, 0.2, 0x00ff0000);
            renderLine(me.body.pos, st.foundTargetPos, 0x00ff0000);
        }
    }
}

void MyStrat::computeDefender()
{
    for (int i = 0; i < 2; ++i)
    {
        SideState &ss = sideState[i];

        F minAttackScore = 1e6;
        F maxAttackScore = -1e6;
        for (MyRobot &en : sim.robots)
        {
            if ((int) en.side == i)
            {
                RobotState &st = robotState[en.id];
                F ascore = -((sim.ball.pos.xz() - en.body.pos.xz()) * P(2, 1)).len() + en.body.pos.z * getSideSign(en.side) * 0.5;
                st.attackScore = ascore;

                if (minAttackScore > ascore)
                {
                    minAttackScore = ascore;
                    ss.defendId = en.id;
                }

                if (maxAttackScore < ascore)
                {
                    maxAttackScore = ascore;
                    ss.attackId = en.id;
                }
            }
        }
    }
}

void MyStrat::compute(const Simulator &_sim)
{
    this->sim = _sim;

    computePredictedBallPos();
    computeDefender();
    currentBallScore = ballScoreFast(sim, sim.robots);
    highDanger = false;
    updateRobotState();
    computeHighDanger();
    if (highDanger)
    {
        //LOG("HD");
        updateRobotState();
    }

    for (MyRobot &me : sim.robots)
    {
        if (me.side != Side::NEG)
            continue;

        RobotState &st = robotState[me.id];

        //LOG("F " << me.id << " " << st.traj.found << " " << st.traj.vel.size());
        if (!st.traj.found)
        {
            WRITE_LOG("GT " << st.foundTargetPos);
            me.action.target_velocity = xz((st.foundTargetPos - me.body.pos).xz().norm(), 0) * st.speed;
            me.action.use_nitro = false;
            me.action.jump_speed = 0;

            computeJump(me);
            optimizeKick(sim, me, enEngageT);
        }
        else
        {
            me.action.use_nitro = false;
            if (st.traj.vel.empty())
            {
                WRITE_LOG("TVJ " << st.traj.lastVel);
                st.nitroTicks = st.traj.nitroTicks;
                me.action.target_velocity = xz(st.traj.lastVel, st.nitroTicks > 0 ? ROBOT_MAX_JUMP_SPEED : 0);
                st.targetVel = me.action.target_velocity;
                me.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
            }
            else
            {
                WRITE_LOG("TV " << st.traj.vel[0] << " L " << st.traj.vel.size() << " F " << st.traj.flyT);
                me.action.target_velocity = xz(st.traj.vel[0], 0);
                me.action.jump_speed = 0;
            }
        }

        bool useNitro = st.nitroTicks > 0;
        bool oldUseNitro = useNitro;

        if (!useNitro)
        {
            st.targetVel = me.body.vel;
            st.targetVel.y -= GRAVITY * TICK_DUR;
        }

        if (oldUseNitro && (!me.touch || me.action.jump_speed > 0) || !oldUseNitro && !me.touch && me.body.vel.y > 1 && me.nitro > 10)
        {
            if (!me.touch)
            {
                F score = computeFlyScore(sim, me.id, enEngageT, st.targetVel, st.nitroTicks);
                F oldScore = score;

                P3 dir = st.targetVel.norm();
                P3 t1 = cross(P3(0, 1, 0), dir).norm();
                P3 t2 = cross(t1, dir);

                P3 bestTargetVel = st.targetVel;

                auto upd = [&](F f, F s1, F s2){
                    P3 newTargetVel = st.targetVel + (dir * f + t1 * s1 + t2 * s2) * 10.0;
                    F newScore = computeFlyScore(sim, me.id, enEngageT, newTargetVel, oldUseNitro ? st.nitroTicks : 5);
                    if (oldUseNitro  && newScore > score || !oldUseNitro && newScore > score + 100)
                    {
                        bestTargetVel = newTargetVel;
                        score = newScore;

                        if (!useNitro)
                        {
                            useNitro = true;
                            //LOG("EN " << oldScore << " " << score);
                        }
                        return true;
                    }
                    return false;
                };

                F f = 0;
                F s1 = 0;
                F s2 = 0;

                if (upd(f, s1 + 0.5, s2))
                {
                    s1 += 0.5;
                    if (upd(f, s1 + 0.5, s2))
                        s1 += 0.5;
                }
                else if (upd(f, s1 - 0.5, s2))
                {
                    s1 -= 0.5;
                    if (upd(f, s1 - 0.5, s2))
                        s1 -= 0.5;
                }

                if (upd(f, s1, s2 + 0.5))
                {
                    s2 += 0.5;
                    if (upd(f, s1, s2 + 0.5))
                        s2 += 0.5;
                }
                else if (upd(f, s1, s2 - 0.5))
                {
                    s2 -= 0.5;
                    if (upd(f, s1, s2 - 0.5))
                        s2 -= 0.5;
                }

                if (upd(f + 0.5, s1, s2))
                {
                    f += 0.5;
                }

                if (upd(f, s1 + 0.25, s2))
                {
                    s1 += 0.25;
                    if (upd(f, s1 + 0.25, s2))
                        s1 += 0.25;
                }
                else if (upd(f, s1 - 0.25, s2))
                {
                    s1 -= 0.25;
                    if (upd(f, s1 - 0.25, s2))
                        s1 -= 0.25;
                }

                if (upd(f, s1, s2 + 0.25))
                {
                    s2 += 0.25;
                    if (upd(f, s1, s2 + 0.25))
                        s2 += 0.25;
                }
                else if (upd(f, s1, s2 - 0.25))
                {
                    s2 -= 0.25;
                    if (upd(f, s1, s2 - 0.25))
                        s2 -= 0.25;
                }


                st.targetVel = bestTargetVel;

                if (score < -1e5)
                {
                    //st.nitroTicks = 0;
                    //LOG("DISABLE NITRO");
                }

                //LOG("SC " << oldScore << " " << score);
            }

            me.action.target_velocity = st.targetVel;
            if (useNitro)
            {
                me.action.use_nitro = true;
                st.nitroTicks--;
                WRITE_LOG("NITRO " << me.id << " " << st.nitroTicks);
            }

            optimizeKick(sim, me, enEngageT);
        }
        else
        {
            st.nitroTicks = 0;
        }
    }
}


