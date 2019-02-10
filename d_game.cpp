#include "d_game.hpp"
#include "d_glm_math.hpp"

constexpr double LIGHT_HEIGHT = 30.0;

SObj rect(const P3 &center, const P3 &t1, const P3 &t2, uint32_t color, const P3 &lightPos, const P3 &gridShift)
{
    SObj res;

    res["type"] = "rect";
    res["gridShift"] = gridShift;
    res["c"] = color;
    res["p"] = center;
    res["ht1"] = t1;
    res["ht2"] = t2;
    res["lp"] = lightPos;

    return res;
}

SObj sphere(const P3 &pos, float r, uint32_t color, const P3 &lightPos)
{
    SObj res;

    res["type"] = "sphere";
    res["p"] = pos;
    res["r"] = r;
    res["c"] = color;
    res["lp"] = lightPos;

    return res;
}

uint32_t rgbF(F r, F g, F b)
{
    unsigned ur = r * 255;
    unsigned ug = g * 255;
    unsigned ub = b * 255;
    return (ur << 24) | (ug << 16) | (ub << 8) | 0xFF;
}


void sendNewTick(TcpClient &client, uint32_t tick)
{
    Obj obj;
    obj.type = "tick";
    obj.props["num"] = tick;
    client.sendObj(obj);
}

uint32_t POS_COLOR = 0x4444ffffu;
uint32_t POS_COLOR_NITRO = 0x0000ffffu;
uint32_t NEG_COLOR = 0xff4444ffu;
uint32_t NEG_COLOR_NITRO = 0xff0000ffu;

void sendMap(TcpClient &tcpClient, const MyArena &arena) {
    P3 lightPos = P3(0, LIGHT_HEIGHT, 0);

    float w05 = arena.width * 0.5;
    float d05 = arena.depth * 0.5;
    float h05 = arena.height * 0.5;
    float gw05 = arena.goal_width * 0.5;
    float gh05 = arena.goal_height * 0.5;
    float gd = arena.goal_depth;

    {
        Obj obj;
        obj.type = "field3d";
        obj.props["minP"] = P(-w05, -d05);
        obj.props["maxP"] = P(w05, d05);
        obj.props["hMin"] = 0.0;
        obj.props["hMax"] = arena.height;
        obj.props["cellSize"] = 1.0;

        tcpClient.sendObj(obj);
    }



    {
        Obj obj;
        obj.type = "static";
        obj.subObjs["floor"] = rect(
                P3(0, 0, 0),
                P3(-w05, 0, 0),
                P3(0, 0, arena.depth * 0.5),
                0xe57f00ffu,
                lightPos,
                P3(0, 0.5, 0)
        );

        obj.subObjs["floorDisk"]["type"] = "disk";
        obj.subObjs["floorDisk"]["p"] = P3(0, 0, 0);
        obj.subObjs["floorDisk"]["n"] = P3(0, 1, 0);
        obj.subObjs["floorDisk"]["r"] = 1.0;
        obj.subObjs["floorDisk"]["c"] = 0x77ffffffu;

        obj.subObjs["sideWalls_r1"] = rect(
                P3(-w05, h05, 0),
                P3(0, h05, 0),
                P3(0, 0, d05),
                0xe57f00ffu,
                lightPos,
                P3(0.5, 0, 0)
        );

        obj.subObjs["sideWalls_r2"] = rect(
                P3(w05, h05, 0),
                P3(0, -h05, 0),
                P3(0, 0, d05),
                0xe57f00ffu,
                lightPos,
                P3(0.5, 0, 0)
        );

        obj.subObjs["wall1_r1"] = rect(
                P3(-w05 * 0.5 - gw05*0.5, h05, d05),
                P3(0, h05, 0),
                P3((w05 - gw05) * 0.5, 0, 0),
                0xe57f00ffu,
                lightPos,
                P3(0.0, 0, 0.5)
        );

        obj.subObjs["wall1_r2"] = rect(
                P3(w05 * 0.5 + gw05*0.5, h05, d05),
                P3(0, h05, 0),
                P3((w05 - gw05) * 0.5, 0, 0),
                0xe57f00ffu,
                lightPos,
                P3(0.0, 0, 0.5)
        );

        obj.subObjs["wall1_r3"] = rect(
                P3(0, gh05 + h05, d05),
                P3(0, h05 - gh05, 0),
                P3(gw05, 0, 0),
                0xe57f00ffu,
                lightPos,
                P3(0, 0, 0.5)
        );

        obj.subObjs["wall2_r1"] = rect(
                P3(-w05 * 0.5 - gw05*0.5, h05, -d05),
                P3(0, h05, 0),
                P3(-(w05 - gw05) * 0.5, 0, 0),
                0xe57f00ffu,
                lightPos,
                P3(0.0, 0, 0.5)
        );

        obj.subObjs["wall2_r2"] = rect(
                P3(w05 * 0.5 + gw05*0.5, h05, -d05),
                P3(0, h05, 0),
                P3(-(w05 - gw05) * 0.5, 0, 0),
                0xe57f00ffu,
                lightPos,
                P3(0.0, 0, 0.5)
        );

        obj.subObjs["wall2_r3"] = rect(
                P3(0, gh05 + h05, -d05),
                P3(0, h05 - gh05, 0),
                P3(-gw05, 0, 0),
                0xe57f00ffu,
                lightPos,
                P3(0, 0, 0.5)
        );



        obj.subObjs["goal1_back"] = rect(
                P3(0, gh05, d05 + gd),
                P3(0, gh05, 0 ),
                P3(gw05, 0, 0),
                POS_COLOR,
                lightPos,
                P3(0.0, 0, 0.5)
        );

        obj.subObjs["goal1_r2"] = rect(
                P3((w05 - gw05), gh05, d05 + gd * 0.5),
                P3(0, -gh05, 0 ),
                P3(0, 0, gd * 0.5),
                POS_COLOR,
                lightPos,
                P3(0.5, 0, 0)
        );

        obj.subObjs["goal1_r3"] = rect(
                P3(-(w05 - gw05), gh05, d05 + gd * 0.5),
                P3(0, gh05, 0 ),
                P3(0, 0, gd * 0.5),
                POS_COLOR,
                lightPos,
                P3(0.5, 0, 0)
        );

        obj.subObjs["goal1_rbottom"] = rect(
                P3(0, 0, d05 + gd * 0.5),
                P3(0, 0, gd*0.5 ),
                P3(gw05, 0, 0),
                POS_COLOR,
                lightPos,
                P3(0, 0.5, 0)
        );

        obj.subObjs["goal1_rtop"] = rect(
                P3(0, gh05 * 2, d05 + gd * 0.5),
                P3(0, 0, gd*0.5 ),
                P3(gw05, 0, 0),
                POS_COLOR,
                lightPos,
                P3(0, 0.5, 0)
        );

//////////

        obj.subObjs["goal2_back"] = rect(
                P3(0, gh05, -(d05 + gd)),
                P3(0, -gh05, 0 ),
                P3(gw05, 0, 0),
                NEG_COLOR,
                lightPos,
                P3(0.0, 0, 0.5)
        );

        obj.subObjs["goal2_r2"] = rect(
                P3((w05 - gw05), gh05, -(d05 + gd * 0.5)),
                P3(0, -gh05, 0 ),
                P3(0, 0, gd * 0.5),
                NEG_COLOR,
                lightPos,
                P3(0.5, 0, 0)
        );

        obj.subObjs["goal2_r3"] = rect(
                P3(-(w05 - gw05), gh05, -(d05 + gd * 0.5)),
                P3(0, gh05, 0 ),
                P3(0, 0, gd * 0.5),
                NEG_COLOR,
                lightPos,
                P3(0.5, 0, 0)
        );

        obj.subObjs["goal2_rbottom"] = rect(
                P3(0, 0, -(d05 + gd * 0.5)),
                P3(0, 0, gd*0.5 ),
                P3(gw05, 0, 0),
                NEG_COLOR,
                lightPos,
                P3(0, 0.5, 0)
        );

        obj.subObjs["goal2_rtop"] = rect(
                P3(0, gh05 * 2, -(d05 + gd * 0.5)),
                P3(0, 0, gd*0.5 ),
                P3(gw05, 0, 0),
                NEG_COLOR,
                lightPos,
                P3(0, 0.5, 0)
        );


        float cr = arena.corner_radius;
        obj.subObjs["c1"]["type"] = "cyl_inner_025";
        obj.subObjs["c1"]["lp"] = lightPos;
        obj.subObjs["c1"]["transf"] = glm::translate(glm::vec3(-w05 + cr*0.5, h05, -d05 + cr * 0.5))
                                      * glm::rotate((float) (M_PI * 0.5), glm::vec3(1, 0, 0))
                                      * glm::scale(glm::vec3(cr, cr, h05 * 2));
        obj.subObjs["c1"]["c"] = 0xe57f00ffu;


        obj.subObjs["c2"]["type"] = "cyl_inner_025";
        obj.subObjs["c2"]["lp"] = lightPos;
        obj.subObjs["c2"]["transf"] = glm::translate(glm::vec3(w05 - cr*0.5, h05, -d05 + cr * 0.5))
                                      * glm::rotate((float) (-M_PI * 0.5), glm::vec3(0, 1, 0))
                                      * glm::rotate((float) (M_PI * 0.5), glm::vec3(1, 0, 0))
                                      * glm::scale(glm::vec3(cr, cr, h05 * 2));
        obj.subObjs["c2"]["c"] = 0xe57f00ffu;


        obj.subObjs["c3"]["type"] = "cyl_inner_025";
        obj.subObjs["c3"]["lp"] = lightPos;
        obj.subObjs["c3"]["transf"] = glm::translate(glm::vec3(w05 - cr*0.5, h05, d05 - cr * 0.5))
                                      * glm::rotate((float) (M_PI), glm::vec3(0, 1, 0))
                                      * glm::rotate((float) (M_PI * 0.5), glm::vec3(1, 0, 0))
                                      * glm::scale(glm::vec3(cr, cr, h05 * 2));
        obj.subObjs["c3"]["c"] = 0xe57f00ffu;


        obj.subObjs["c4"]["type"] = "cyl_inner_025";
        obj.subObjs["c4"]["lp"] = lightPos;
        obj.subObjs["c4"]["transf"] = glm::translate(glm::vec3(-w05 + cr*0.5, h05, d05 - cr * 0.5))
                                      * glm::rotate((float) (M_PI * 0.5), glm::vec3(0, 1, 0))
                                      * glm::rotate((float) (M_PI * 0.5), glm::vec3(1, 0, 0))
                                      * glm::scale(glm::vec3(cr, cr, h05 * 2));
        obj.subObjs["c4"]["c"] = 0xe57f00ffu;


        float br = arena.bottom_radius;
        obj.subObjs["bc1"]["type"] = "cyl_inner_025";
        obj.subObjs["bc1"]["lp"] = lightPos;
        obj.subObjs["bc1"]["transf"] = glm::translate(glm::vec3(-w05 + br*0.5, br * 0.5, 0))
                                       //* glm::rotate((float) (M_PI * 0.5), P3(1, 0, 0))
                                       * glm::scale(glm::vec3(br, br, d05 * 2));
        obj.subObjs["bc1"]["c"] = 0xe57f00ffu;

        obj.subObjs["bc2"]["type"] = "cyl_inner_025";
        obj.subObjs["bc2"]["lp"] = lightPos;
        obj.subObjs["bc2"]["transf"] = glm::translate(glm::vec3(w05 - br*0.5, br * 0.5, 0))
                                       * glm::rotate((float) (M_PI), glm::vec3(0, 1, 0))
                                       * glm::scale(glm::vec3(br, br, d05 * 2));
        obj.subObjs["bc2"]["c"] = 0xe57f00ffu;


        tcpClient.sendObj(obj);
    }
}

void sendObjects(TcpClient &tcpClient, const Simulator &sim)
{
    sendNewTick(tcpClient, 1);

    P3 lightPos = P3(0, LIGHT_HEIGHT, 0);

    for (const MyRobot& robot : sim.robots) {

        double x = robot.body.pos.x;
        double y = robot.body.pos.y;
        double z = robot.body.pos.z;
        double r = robot.body.rad;

        uint32_t color = robot.action.use_nitro ? (robot.side == Side::NEG ? NEG_COLOR_NITRO : POS_COLOR_NITRO) : (robot.side == Side::NEG ? NEG_COLOR : POS_COLOR);
        //if (robot.ballTouch)
        //    color = 0x22ff22ff;
        Obj obj;

        std::ostringstream name;
        name << "robot" << robot.id << (robot.side == Side::NEG ? "_N" : "_P");
        obj.type = name.str();
        obj.props["pos"] = robot.body.pos;
        obj.props["vel"] = robot.body.vel;
        obj.props["touch"] = robot.touch ? "true" : "false";
        obj.props["touchN"] = robot.touch_normal;
        obj.props["nitro"] = robot.nitro;
        obj.props["id"] = (unsigned) robot.id;
        obj.props["R"] = robot.body.rad;
        //obj.props["dr"] = robot.body.radius_change_speed;

        obj.subObjs["sph"] = sphere(P3(x, y, z), r, color, lightPos);

        obj.subObjs["disk"]["type"] = "disk";
        obj.subObjs["disk"]["p"] = P3(x, 0, z);
        obj.subObjs["disk"]["n"] = P3(0, 1, 0);
        obj.subObjs["disk"]["r"] = r;
        obj.subObjs["disk"]["c"] = color;

        obj.subObjs["l"]["type"] = "line3d";
        obj.subObjs["l"]["p1"] = P3(x, y, z);
        obj.subObjs["l"]["p2"] = P3(x, 0, z);
        obj.subObjs["l"]["c"] = color;

        if (sim.nitro_packs.size() > 0)
        {
            F W = 0.8;
            F H = 0.15;
            F w = W * robot.nitro / MAX_NITRO_AMOUNT;

            obj.subObjs["npy"] = rect(
                    P3(robot.body.pos + P3(0, 1, 0) * (ROBOT_RADIUS + H * 0.7) - P3(-W * 0.5f + w * 0.5f, 0, 0)),
                    P3(w * 0.5f, 0, 0),
                    P3(0, H * 0.5f, 0) * (robot.body.pos.z < 0 ? 1 : -1),
                    0x00ff00ff,
                    lightPos,
                    P3(0, 0, 0)
            );

            obj.subObjs["npx"] = rect(
                    P3(robot.body.pos + P3(0, 1, 0) * (ROBOT_RADIUS + H * 1.2) - P3(-W * 0.5f + w * 0.5f, 0, 0)),
                    P3(w * 0.5f, 0, 0),
                    P3(0, 0, -H * 0.5f),
                    0x00ff00ff,
                    lightPos,
                    P3(0, 0, 0)
            );

            F nw = W - w;
            obj.subObjs["nny"] = rect(
                    P3(robot.body.pos + P3(0, 1, 0) * (ROBOT_RADIUS + H * 0.7) - P3(W * 0.5f - nw * 0.5f, 0, 0)),
                    P3(nw * 0.5f, 0, 0),
                    P3(0, H * 0.5f, 0) * (robot.body.pos.z < 0 ? 1 : -1),
                    0xff0000ff,
                    lightPos,
                    P3(0, 0, 0)
            );

            obj.subObjs["nnx"] = rect(
                    P3(robot.body.pos + P3(0, 1, 0) * (ROBOT_RADIUS + H * 1.2) - P3(W * 0.5f - nw * 0.5f, 0, 0)),
                    P3(nw * 0.5f, 0, 0),
                    P3(0, 0, -H * 0.5f),
                    0xff0000ff,
                    lightPos,
                    P3(0, 0, 0)
            );
        }

        tcpClient.sendObj(obj);
    }

    {
        double x = sim.ball.pos.x;
        double y = sim.ball.pos.y;
        double z = sim.ball.pos.z;
        double r = sim.ball.rad;

        Obj obj;
        obj.type = "ball";
        obj.props["pos"] = sim.ball.pos;
        obj.props["vel"] = sim.ball.vel;

        obj.subObjs["sph"] = sphere(P3(x, y, z), r, 0xffff00ffu, lightPos);

        obj.subObjs["disk"]["type"] = "disk";
        obj.subObjs["disk"]["p"] = P3(x, 0, z);
        obj.subObjs["disk"]["n"] = P3(0, 1, 0);
        obj.subObjs["disk"]["r"] = r;
        obj.subObjs["disk"]["c"] = 0xffff00ffu;

        obj.subObjs["l"]["type"] = "line3d";
        obj.subObjs["l"]["p1"] = P3(x, y, z);
        obj.subObjs["l"]["p2"] = P3(x, 0, z);
        obj.subObjs["l"]["c"] = 0xffff00ffu;

        tcpClient.sendObj(obj);
    }

    for (const MyNitroPack &pack : sim.nitro_packs)
    {
        uint32_t color = pack.respawn_ticks > 0 ? 0x888888 : 0x22ff22ff;

        Obj obj;
        obj.type = "nitro_" + std::to_string((int) pack.pos.x) + "_" + std::to_string((int) pack.pos.z);
        obj.props["pos"] = pack.pos;
        obj.props["respawn"] = (uint32_t) pack.respawn_ticks;

        obj.subObjs["sph"] = sphere(pack.pos, pack.respawn_ticks > 0 ? 0.1f : pack.rad, color, lightPos);
        if (pack.respawn_ticks > 0)
        {
            P3 pos = pack.pos;
            pos.y *= pack.respawn_ticks / NITRO_RESPAWN_TICKS;
            obj.subObjs["resp"] = sphere(pos, pack.respawn_ticks > 0 ? 0.05f : pack.rad, color, lightPos);
        }

        tcpClient.sendObj(obj);
    }
}

Obj saveSimAsObj(const Simulator &sim) {
    Obj obj;
    obj.subObjs["ball"]["pos"] = sim.ball.pos;
    obj.subObjs["ball"]["vel"] = sim.ball.vel;

    obj.props["robotsN"] = (uint32_t) sim.robots.size();

    int i = 0;
    for (const MyRobot &r : sim.robots)
    {
        std::string robotName = "robot_" + std::to_string(i);
        ++i;

        obj.subObjs[robotName]["id"] = (uint32_t) r.id;
        obj.subObjs[robotName]["pos"] = r.body.pos;
        obj.subObjs[robotName]["vel"] = r.body.vel;
        obj.subObjs[robotName]["radius_change_speed"] = r.body.radius_change_speed;
        obj.subObjs[robotName]["side"] = r.side == Side::NEG ? 0u : 1u;
        obj.subObjs[robotName]["touch"] = r.touch ? 1u : 0u;
        obj.subObjs[robotName]["touch_normal"] = r.touch_normal;
        obj.subObjs[robotName]["nitro"] = r.nitro;
    }

    obj.props["nitroN"] = (uint32_t) sim.nitro_packs.size();

    for (const MyNitroPack &r : sim.nitro_packs)
    {
        std::string nitroName = "nitro_" + std::to_string(i);
        ++i;

        obj.subObjs[nitroName]["pos"] = r.pos;
        obj.subObjs[nitroName]["respawn_ticks"] = (uint32_t) r.respawn_ticks;
    }

    obj.props["curTick"] = (uint32_t) sim.curTick;

    return obj;
}

Simulator loadSimFromObj(MyArena *arena, const Obj &obj)
{
    Simulator sim = createSim(arena, 0, false);

    sim.ball.pos = getP3("pos", obj.subObjs.find("ball")->second);
    sim.ball.vel = getP3("vel", obj.subObjs.find("ball")->second);

    uint32_t robotsN = getInt("robotsN", obj.props);

    sim.robots.clear();
    sim.robots.reserve(robotsN);
    for (size_t i = 0; i < robotsN; ++i)
    {
        std::string robotName = "robot_" + std::to_string(i);

        const SObj &robotObj = obj.subObjs.find(robotName)->second;
        int id = (int) getInt("id", robotObj);
        Side side = getInt("side", robotObj) == 0 ? Side::NEG : Side::POS;
        MyRobot r = createRobot(id, side, P(0, 0));
        r.body.pos = getP3("pos", robotObj);
        r.body.vel = getP3("vel", robotObj);
        r.body.radius_change_speed = getDouble("radius_change_speed", robotObj);
        r.touch = getInt("touch", robotObj) == 1;
        r.touch_normal = getP3("touch_normal", robotObj);
        r.nitro = getDouble("nitro", robotObj);

        sim.robots.push_back(r);
    }

    uint32_t nitroN = getInt("nitroN", obj.props);

    sim.nitro_packs.clear();
    sim.nitro_packs.reserve(nitroN);
    for (size_t i = 0; i < nitroN; ++i)
    {
        std::string nitroName = "nitro_" + std::to_string(i);

        const SObj &nitroObj = obj.subObjs.find(nitroName)->second;

        MyNitroPack n;
        n.pos = getP3("pos", nitroObj);
        n.rad = NITRO_PACK_RADIUS;
        n.respawn_ticks = getInt("respawn_ticks", nitroObj);

        sim.nitro_packs.push_back(n);
    }

    sim.curTick = getInt("curTick", obj.props);

    return sim;
}
