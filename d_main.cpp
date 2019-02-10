#include <iostream>

#include "Simulator.hpp"
#include "Simulator2d.hpp"
#include "d_tcpclient.hpp"
#include <set>
#include "d_game.hpp"
#include "d_oldversions.hpp"

using namespace std::chrono_literals;

#include "RemoteProcessClient.h"
#include "MyStrategy.h"

bool repeater = false;
bool render = false || repeater;
int SEED = 75000;
int gameCount = 400;
int robotsN = 3;
bool enableNitro = true;
constexpr double LIGHT_HEIGHT = 30.0;

TcpClient client;

void renderSphere(const P3 &c, F r, uint32_t color)
{
    if (render)
    {
        Obj obj;
        obj.type = "dsph";
        obj.subObjs["sph"] = sphere(c, r, color, P3(0, LIGHT_HEIGHT, 0));
        client.sendObj(obj);
    }
}

void renderLine(const P3 &p1, const P3 &p2, uint32_t color)
{
    if (render)
    {
        Obj obj;
        obj.type = "dline";
        obj.subObjs["l"]["type"] = "line3d";
        obj.subObjs["l"]["p1"] = p1;
        obj.subObjs["l"]["p2"] = p2;
        obj.subObjs["l"]["c"] = color;
        client.sendObj(obj);
    }
}

void renderLineV(const std::vector<P3> &l, uint32_t color)
{
    if (render)
    {
        Obj obj;
        obj.type = "dline";
        obj.subObjs["l"]["type"] = "line3d";
        obj.subObjs["l"]["c"] = color;

        int i = 0;
        for (const P3 &p : l)
        {
            obj.subObjs["l"]["p" + std::to_string(++i)] = p;
        }

        client.sendObj(obj);
    }
}

void writeLog(const std::string &str)
{
    if (render)
    {
        Obj obj;
        obj.type = "log";
        obj.props["text"] = str;
        client.sendObj(obj);
    }
}

int mainRun() {
    std::unique_ptr<Strategy> strategy(new MyStrategy);
    std::unique_ptr<model::Game> game;
    std::unordered_map<int, model::Action> actions;

    RemoteProcessClient remoteProcessClient("127.0.0.1", 31001);



    if (render)
        client.run();

    remoteProcessClient.write_token("0000000000000000");
    std::unique_ptr<model::Rules> rules = remoteProcessClient.read_rules();

    MyArena arena = convertMyArena(*rules);
    if (render)
        sendMap(client, arena);

    Simulator sim;

    int startTick = 2070;
    int endTick = 2150;
    int tick = 0;
    while ((game = remoteProcessClient.read_game()) != nullptr) {
        ++tick;
        if (tick < startTick)
            continue;

        if (tick > endTick)
            break;

        actions.clear();
        for (const model::Robot& robot : game->robots) {
            if (robot.is_teammate) {
                strategy->act(robot, *rules, *game, actions[robot.id]);
            }
        }
        remoteProcessClient.write(actions);

        sim = convertSimulator(&arena, *game);
        if (render)
        {
            sendObjects(client, sim);

            Simulator testSim = sim;
            testSim.robots.clear();
            for (int t = 0; t < 100; ++t)
            {
                testSim.tickFast();
                renderSphere(testSim.ball.pos, 0.04, 0xffffffffu);
            }

            for (const MyRobot &r : sim.robots)
            {
                if (!r.touch)
                {
                    Simulator testSim = sim;
                    filterVector(testSim.robots, [&r](const MyRobot &rr){return rr.id != r.id;});

                    for (int t = 0; t < 100 && !testSim.robots[0].touch; ++t)
                    {
                        testSim.tickFast();
                        renderSphere(testSim.robots[0].body.pos, 0.04, 0xffffffffu);
                    }
                }
                else
                {
                    Simulator testSim = sim;
                    filterVector(testSim.robots, [&r](const MyRobot &rr){return rr.id != r.id && rr.touch;});

                    int myInd = testSim.indexOfRobot(r.id);

                    testSim.robots[myInd].action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                    testSim.tick();
                    renderSphere(testSim.robots[myInd].body.pos, 0.05, 0xffffffffu);

                    for (int t = 0; t < 100 && !testSim.robots[myInd].touch; ++t)
                    {
                        testSim.tickFast();
                        renderSphere(testSim.robots[myInd].body.pos, 0.04, 0xffffffffu);
                        renderSphere(testSim.ball.pos, 0.07, 0x00ffffffu);
                    }
                }
            }

            for (auto &p : actions)
            {
                if (p.second.jump_speed > 0)
                {
                    Obj obj;
                    obj.type = "log";
                    obj.props["text"] = "Jump " + std::to_string(p.first) + " " + std::to_string(p.second.jump_speed);
                    client.sendObj(obj);
                }
            }
        }
    }
    return 0;
}

void saveSim(const Simulator &sim)
{
    //return;
    //if (!saveSims)
    //    return;

    Obj obj = saveSimAsObj(sim);

    std::ofstream f("sv1.bin", std::ios::binary | std::ios::app);
    writeObj(f, obj);
}

std::vector<Simulator> readSims(MyArena *arena)
{
    std::ifstream f("sv1.bin", std::ios::binary);

    std::vector<Simulator> res;

    while(1)
    {
        Obj obj = readObj(f);
        if (!f)
            break;

        Simulator sim = loadSimFromObj(arena, obj);
        res.push_back(sim);
    }

    return res;
}

void mainRunner()
{
    srand(SEED);

    if (render)
        client.run();

    MyArena arena = createArena();
    if (render)
        sendMap(client, arena);

//////////////////////////////////////



    //return;
    ///////////////////////

    int N = 0;
    int P = 0;
    int NN = 0;
    int PP = 0;

    int nn, pp;

    double t1 = 0;
    double t2 = 0;

    int totalTick = 0;
    for (int i = 0; i < gameCount; ++i)
    {
        if (i % 5 == 0)
        {
            nn = pp = 0;
        }

        Simulator sim = createSim(&arena, robotsN, enableNitro);

        //std::vector<Simulator> sims = readSims(&arena);
        //sim = sims[0];


        // V10/V8 = 144/56
        // V12/V8 = 156/44
        // V13/V8 = 174/26
        // V13/V10 = 154/46
        // V16/V13 = 138/62  	307	185384	t1: 223219 t2: 197006	| 33 7
        // V20/V19 = 138/93 254 267465 t1: 1.02344e+06 t2: 953483 | 32 14
        // V20/V17 = 58/20	565	83771	t1: 314511 t2: 257606	| 14 1
        // V21/V19 = 226/174	1171	412068	t1: 612497 t2: 523304	| 55 25
        // V23/V22 = 149 101	1651	388948	t1: 762913 t2: 723725	| 38 12
        // V23/V19 = 86 26	502	122137	t1: 458950 t2: 427923	| 21 1 | R3
        // V25/V24 = 36/28	477	107972	t1: 443152 t2: 459064	| 8 4 | R3
        // V26/V24 = 32/14	298	82287	t1: 338947 t2: 353921	| 8 1 | R3
        // V28/V27 = 73 62	951	478324	t1: 2.05545e+06 t2: 2.02125e+06	| 15 12 | R3
        MyStrat strat1;
        //EmptyStrat strat2;
        //QuickStartGuy strat2;
        //stratV3::MyStrat strat2;
        //stratV7::MyStrat strat2;
        //stratV8::MyStrat strat2;
        //stratV9::MyStrat strat2;
        //stratV10::MyStrat strat2;
        //stratV11::MyStrat strat2;
        //stratV12::MyStrat strat2;
        //stratV13::MyStrat strat2;
        //stratV15::MyStrat strat2;
        //stratV16::MyStrat strat2;
        //stratV17::MyStrat strat2;
        //stratV19::MyStrat strat2;
        //stratV22::MyStrat strat2;
        //stratV23::MyStrat strat2;
        //stratV24::MyStrat strat2;
        //stratV25::MyStrat strat2;
        //stratV26::MyStrat strat2;
        //stratV27::MyStrat strat2;
        //stratV28::MyStrat strat2;
        stratV29::MyStrat strat2;

        int tick = 0;
        for (; tick < 18000; ++tick)
        {
            if (render)
                sendObjects(client, sim);

            for (MyRobot &r : sim.robots)
            {
                r.action = MyAction();
            }

            Simulator mirroredSim = mirrorSim(sim);
            {
                TimeMeasure t(t1);
                strat1.compute(sim);
            }
            {
                TimeMeasure t(t2);
                strat2.compute(mirroredSim);
            }

            /*if (render)
            {
                Obj obj;
                obj.type = "pr";
                int bi;
                for (const Ball &b : strat1.predictedBallPos)
                {
                    obj.subObjs["sph" + std::to_string(++bi)] = sphere(b.pos, 0.1, 0xffff00ffu, P3(0, 30, 0));
                }
                client.sendObj(obj);
            }*/

            copyActions(sim, strat1.sim);
            copyMirroredActions(sim, strat2.sim);

            sim.tick();

            if (sim.scoreSide != Side::NONE)
            {
                if (sim.scoreSide == Side::NEG)
                {
                    ++N;
                    ++nn;
                }
                else
                {
                    ++P;
                    ++pp;
                }

                if (i % 5 == 4)
                {
                    if (i > 0)
                    {
                        if (nn > pp)
                            ++NN;
                        else if (nn < pp)
                            ++PP;
                    }
                }

                break;
            }

            //LOG("T " << tick);
            /*if (totalTick + tick == 2895)
            {
                saveSim(sim);
                return;
            }*/
        }

        totalTick += tick;
        LOG(N << " " << P << "\t" << tick << "\t" << totalTick << "\tt1: " << t1 << " t2: " << t2 << "\t| " << NN << " " << PP << " | R" << robotsN);
    }
}

CollisionInfo dan_to_arena(P3 point, const MyArena &arena);


F ballScore1(const Simulator &ssim)
{
    const Ball &ball = ssim.ball;
    F sscore = ball.vel.z;

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
            F goalScore = 1000 + (testSim.ball.pos.y - BALL_RADIUS) * 100;

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

        //if (t == 10)
        //    testSim.robots.clear();
    }

    renderLineV(renderPoints, 0xff444400);

    return sscore;
}

inline F myRandom(F vmin, F vmax)
{
    return rand() / (F) RAND_MAX * (vmax - vmin) + vmin;
}






void renderPt(P p, uint32_t c)
{
    if (render)
    {
        Obj obj;
        obj.subObjs["c"]["type"] = "circle";
        obj.subObjs["c"]["r"] = 0.05;
        obj.subObjs["c"]["p"] = p + P(50, 50);
        obj.subObjs["c"]["c"] = c;
        client.sendObj(obj);
    }
}

F optimizeSpeed(F deltaL, F curSpdF, F curSpdB, int T);
bool findTraj(Trajectory2d &traj2, const P &targetPos, const P &targetVel, int targetT)
{
    P middle;
    F minDist = 1e6;

    for (int brakingTicks = 0; brakingTicks < targetT/2; ++brakingTicks)
    {
        sendNewTick(client, brakingTicks);
        for (int iter = 0; iter < (brakingTicks > 0 ? 1 : 2); ++ iter)
        {
            traj2.found = false;
            Robot2d robot2df = traj2.robot2d;
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
                renderPt(robot2db.pos, 0xaa0000ff);
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
                renderPt(robot2df.pos, 0x00aa00ff);
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
                traj2.vel.insert(traj2.vel.end(), vfs.begin(), vfs.end());
                traj2.vel.insert(traj2.vel.end(), vbs.rbegin(), vbs.rend());
                traj2.found = true;

                //if (iter > 0)
                //    LOG("FFFFFFFFFF");
                return true;
            }
        }
    }

    return false;
}

void sendNewTick(TcpClient &client, uint32_t tick);

void testSim2()
{
    MyArena arena = createArena();
    Simulator sim = createSim(&arena, 0, true);

    MyRobot r = createRobot(1, Side::NEG, P(5, 0));
    r.nitro = 50;
    r.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
    r.action.use_nitro = false;
    r.action.target_velocity = P3(0, 15, 0);
    sim.robots.push_back(r);

    int nitroT = 22;
    for (int i = 0; i <= 35 + nitroT; ++i)
    {
        LOG("jumpAttrs[3].push_back({" + std::to_string(sim.robots[0].body.pos.y) + ", " + std::to_string(sim.robots[0].body.vel.y) + "});");

        if (i < nitroT)
        {
            sim.robots[0].action.use_nitro = true;
        }
        else
        {
            sim.robots[0].action.use_nitro = false;
        }
        sim.tick();
    }

    LOG("N " << sim.robots[0].nitro);

    /*if (render)
        client.run();

    MyArena arena = createArena();
    if (render)
        sendMap(client, arena);

    Simulator sim = createSim(&arena, 0, false);
    MyRobot r = createRobot(1, Side::NEG, P(0, 0));
    sim.robots.push_back(r);

    r = createRobot(2, Side::NEG, P(2.5, 0));
    sim.robots.push_back(r);

    sim.ball.pos = P3(0, 2, -30);
    sim.ball.vel = P3(5, 0, -5);

    MyStrat strat1;

    int tick = 0;
    for (; tick < 200; ++tick)
    {
        if (render)
            sendObjects(client, sim);

        for (MyRobot &r : sim.robots)
        {
            r.action = MyAction();
        }

        strat1.compute(sim);

        copyActions(sim, strat1.sim);

        sim.tick();

        if (sim.scoreSide != Side::NONE)
        {
            break;
        }
    }*/
}

void testSim()
{
    /*Trajectory2d traj2;
    traj2.robot2d.pos = P(0, 0);
    traj2.robot2d.vel = P(1.62536,12.9177);

    bool bb = traj2.find(P(0.000725269,5.07461), P(1.08687,29.9803), 15);
    LOG("R " << bb);

    //NF TP (0.000725269,5.07461) TV (1.08687,29.9803) T 15 FV (1.62536,12.9177)
    return;*/

    if (render)
        client.run();

    MyArena arena = createArena();
    if (render)
        sendMap(client, arena);

    Simulator sim = createSim(&arena, 0, false);
    MyRobot r = createRobot(1, Side::NEG, P(0, -39));
    sim.robots.push_back(r);

    int N = 0;
    int totalTick = 0;

    std::vector<Simulator> sims = readSims(&arena);
    for (int test = 0; test < 2500; ++test)
    //int test = 0;
    //for (Simulator tsim : sims)
    {
        /*++test;
        if (test > 20)
            break;*/

        Simulator tsim = sim;
        tsim.ball.pos.y = myRandom(2, 10);
        tsim.ball.pos.x = myRandom(-25, 25);
        tsim.ball.pos.z = myRandom(-5, 5);

        P3 dir = (P3(myRandom(-15, 15), myRandom(0, 10), -41) - tsim.ball.pos).norm()*random(35, 45);
        tsim.ball.vel = dir;
        tsim.ball.vel.y = random(-5, 30);

        tsim.robots[0].body.vel = xz(P(random(0, M_PI * 2)) * 15, 0);

        Simulator svSim = tsim;

        MyStrat strat1;

        int tick = 0;
        for (; tick < 200; ++tick)
        {
            if (render)
                sendObjects(client, tsim);

            for (MyRobot &r : tsim.robots)
            {
                r.action = MyAction();
            }

            strat1.compute(tsim);

            copyActions(tsim, strat1.sim);

            tsim.tick();

            if (tsim.scoreSide != Side::NONE)
            {
                if (tsim.scoreSide == Side::POS)
                    ++N;

                break;
            }

            if (tsim.ball.pos.z > -35 && tsim.ball.vel.z > 0)
                break;


            /*for (int dirI = -6; dirI <= 6; ++dirI)
            {
                P dir = P(2.0 * M_PI * dirI / 24.0 - M_PI * 0.5);

                Simulator sim2 = tsim;

                for (int j = 0; j < 60; ++j)
                {
                    sim2.robots[0].action.jump_speed = 0;
                    sim2.robots[0].action.target_velocity = xz(dir * 30, 0);
                    if (sim2.robots[0].body.pos.y > 3)
                        sim2.robots[0].action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                    sim2.tickFast();

                    renderSphere(sim2.robots[0].body.pos, 0.1, 0x00ff0000);
                }
            }*/

            //LOG("T " << tick);
        }

        /*if (tsim.scoreSide == Side::POS)
        {
            saveSim(svSim);
        }*/

        totalTick += tick;
        LOG("R " << (int) tsim.scoreSide << " " << N << "/" << test << " ts " << totalTick);

    }
}




#ifdef CUSTOM_RUNNER
int main() {

    //MyArena arena = createArena();
    //LOG("P " << dan_to_arena(V3(14.8249,1,-37.1747), arena).distance);
    //LOG("P " << dan_to_arena(V3(14.8272,1,-37.1764), arena).distance);


    //testSim2();
    //return 0;




    if (!repeater)
        mainRunner();
    else
        mainRun();
    //mainCustom();

    /*TcpClient client;
    client.run();

    FieldGeometry geom;
    buildGeometry(geom);

    Simulator sim;
    sim.geometry = &geom;
    sim.ball.pos = V3(30, 10, 1);
    sim.ball.rad = 1.0;
    sim.ball.mass = 1.0;
    sim.ball.vel = V3(0, 0, 0);

    sim.players.push_back(Player());
    Player &p = sim.players.back();
    p.team = 0;
    p.body.pos = V3(10, 10, 0.5);
    p.body.rad = 0.5;
    p.body.mass = 1.5;
    p.body.vel = V3(0);

    sendField(client);

    bool pause = true;

    auto step = [&](int tick){
        sendNewTick(client, tick);
        client.sendObj(ballObj(sim.ball));
        sim.ball.accel = V3(0, 0, -GRAVITY);

        for (const Player & p : sim.players)
        {
            client.sendObj(playerObj(p));
        }

        sim.step();
    };

    bool running = true;

    std::set<uint32_t > pressed;

    for (int tick = 0; tick < 100000 && running; )
    {
        Obj msg;
        while (client.inMsgs.tryPop(msg, pause ? 16ms : 0ms))
        {
            //LOG("GOT " << msg.type);
            if (msg.type.empty())
            {
                running = false;
                break;
            }

            if (msg.type == "keyPress")
            {
                if (msg.getIntProp("key") == KEYS::PAUSE)
                {
                    pause = !pause;
                }

                pressed.insert(msg.getIntProp("key"));
            }

            if (msg.type == "keyRelease")
            {
                pressed.erase(msg.getIntProp("key"));
            }
        }

        if (!pause || pressed.count(KEYS::RIGHT))
        {
            V3 accel = V3(0, 0, -GRAVITY);

            float maxAccel = 0.5 * DT;

            if (pressed.count(KEYS::LEFT))
            {
                accel.x -= maxAccel;
            }

            if (pressed.count(KEYS::RIGHT))
            {
                accel.x += maxAccel;
            }

            if (pressed.count(KEYS::UP))
            {
                accel.y += maxAccel;
            }

            if (pressed.count(KEYS::DOWN))
            {
                accel.y -= maxAccel;
            }

            if (accel.x == 0)
            {
                accel.x = -p.body.vel.x;
                if (std::abs(accel.x) > maxAccel)
                    accel.x = std::copysign(maxAccel, accel.x);
            }

            if (accel.y == 0)
            {
                accel.y = -p.body.vel.y;
                if (std::abs(accel.y) > maxAccel)
                    accel.y = std::copysign(maxAccel, accel.y);
            }

            p.body.accel = accel;

            ++tick;
            step(tick);
            usleep(16000);
        }
    }
    return 0;*/
}

#endif
