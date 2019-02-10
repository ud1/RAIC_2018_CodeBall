#ifndef RAIC2018_MYSTRAT_HPP
#define RAIC2018_MYSTRAT_HPP

#include "Simulator2d.hpp"
#include <vector>
#include <map>

struct MyStratBase
{
    virtual void compute(const Simulator &sim) = 0;
    Simulator sim;
};

#ifdef ENABLE_LOGGING
void renderSphere(const P3 &c, F r, uint32_t color);
void renderLine(const P3 &p1, const P3 &p2, uint32_t color);
void renderLineV(const std::vector<P3> &l, uint32_t color);
void writeLog(const std::string &str);

#define WRITE_LOG(x) {std::ostringstream oss; oss << x; writeLog(oss.str());}

#else
#define renderSphere(c, r, color);
#define renderLine(p1, p2, color);
#define renderLineV(l, color);
#define writeLog(str);

#define WRITE_LOG(x);
#endif




struct RobotState
{
    P3 foundTargetPos;
    size_t targetT;
    F speed, height;
    Trajectory2d traj;
    int nitroTicks;
    P3 targetVel;
    F trajScore;
    F attackScore;
    int jumpT = -1;
    bool targetPointFound;
};

struct SideState
{
    int defendId = -1;
    int attackId = -1;
};

struct MyStrat : MyStratBase
{
    std::map<int, RobotState> robotState;
    SideState sideState[2];
    int enEngageT;
    F currentBallScore = 0;
    bool highDanger = false;

    void computeDefender();
    void computeJump(MyRobot &me);
    void updateRobotState();
    void computeHighDanger();
    void computePredictedBallPos();
    F computePP(const P &pos, int myId, int T);
    virtual void compute(const Simulator &sim) override;
    //std::vector<Ball> predictedBallPos;
};


#endif //RAIC2018_MYSTRAT_HPP
