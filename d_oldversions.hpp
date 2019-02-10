#ifndef RAIC2018_D_OLDVERSIONS_HPP
#define RAIC2018_D_OLDVERSIONS_HPP

#include "MyStrat.hpp"

struct EmptyStrat : MyStratBase
{
    virtual void compute(const Simulator &_sim) override
    {
        this->sim = _sim;

        for (MyRobot &me : sim.robots)
        {
            if (me.side != Side::NEG)
                continue;

            me.action.jump_speed = 0;
            me.action.target_velocity = P3(0);
            me.action.use_nitro = false;
        }
    }
};

struct QuickStartGuy : MyStratBase
{
    virtual void compute(const Simulator &sim) override;
};



namespace stratV24 {
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

        void computeDefender();
        void computeJump(MyRobot &me);
        void updateRobotState();
        void computePredictedBallPos();
        F computePP(const P &pos, int myId, int T);
        virtual void compute(const Simulator &sim) override;
        //std::vector<Ball> predictedBallPos;
    };
}

namespace stratV25 {
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

        void computeDefender();
        void computeJump(MyRobot &me);
        void updateRobotState();
        void computePredictedBallPos();
        F computePP(const P &pos, int myId, int T);
        virtual void compute(const Simulator &sim) override;
        //std::vector<Ball> predictedBallPos;
    };
}

namespace stratV26 {
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

        void computeDefender();
        void computeJump(MyRobot &me);
        void updateRobotState();
        void computePredictedBallPos();
        F computePP(const P &pos, int myId, int T);
        virtual void compute(const Simulator &sim) override;
        //std::vector<Ball> predictedBallPos;
    };
}

namespace stratV27 {
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
}

namespace stratV28 {
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
}


namespace stratV29 {
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

}
#endif //RAIC2018_D_OLDVERSIONS_HPP
