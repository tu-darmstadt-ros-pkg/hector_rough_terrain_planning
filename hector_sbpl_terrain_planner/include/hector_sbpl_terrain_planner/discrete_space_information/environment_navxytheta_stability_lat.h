//Header for new environment, analogous to https://github.com/sbpl/sbpl/tree/master/src/include/sbpl

#ifndef __ENVIRONMENT_NAVXYTHETA_STABILITY_LAT_H_
#define __ENVIRONMENT_NAVXYTHETA_STABILITY_LAT_H_


#include <cstdio>
#include <vector>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#define XXX_MAXACTIONSWIDTH 9

class CMDPACTION;
class CMDPSTATE;
class MDPConfig;

typedef struct ENV_NAVXYTHETASTAB_CONFIG
{
    //parameters that are read from the configuration file
    unsigned int StartX1;
    unsigned int StartX2;
    unsigned int StartX3;
    unsigned int StartX4;
    unsigned int GoalX1;
    unsigned int GoalX2;
    unsigned int GoalX3;
    unsigned int GoalX4;

    //derived and initialized elsewhere parameters
} EnvNAVXYTHETASTABConfig_t;

typedef struct ENVNAVXYTHETASTABHASHENTRY
{
    int stateID;
    unsigned int X1;
    unsigned int X2;
    unsigned int X3;
    unsigned int X4;
} EnvNAVXYTHETASTABHashEntry_t;

typedef struct
{
    int startstateid;
    int goalstateid;

    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvNAVXYTHETASTABHashEntry_t*>* Coord2StateIDHashTable;

    //vector that maps from stateID to coords
    std::vector<EnvNAVXYTHETASTABHashEntry_t*> StateID2CoordTable;

    //any additional variables
} EnvironmentNAVXYTHETASTAB_t;

/** \brief this is just an example of environment and can be used (copy and
* paste) for creating a more complex environment
*/
class   EnvironmentNAVXYTHETASTAB : public EnvironmentNAVXYTHETALAT
{
public:
    /**
* \brief see comments on the same function in the parent class
*/
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual int GetGoalHeuristic(int stateID);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual int GetStartHeuristic(int stateID);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual void SetAllPreds(CMDPSTATE* state);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual int SizeofCreatedEnv();

    /**
* \brief see comments on the same function in the parent class
*/
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
* \brief see comments on the same function in the parent class
*/
    virtual void PrintEnv_Config(FILE* fOut);

    ~  EnvironmentNAVXYTHETASTAB() { }

protected:
    //member variables
    EnvNAVXYTHETASTABConfig_t EnvNAVXYTHETASTABCfg;
    EnvironmentNAVXYTHETASTAB_t EnvNAVXYTHETASTAB;

    virtual void ReadConfiguration(FILE* fCfg);

    virtual void InitializeEnvConfig();

    virtual unsigned int GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4);

    virtual void PrintHashTableHist();

    virtual EnvNAVXYTHETASTABHashEntry_t* GetHashEntry(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4);

    virtual EnvNAVXYTHETASTABHashEntry_t* CreateNewHashEntry(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4);

    virtual void CreateStartandGoalStates();

    virtual void InitializeEnvironment();

    virtual void AddAllOutcomes(unsigned int SourceX1, unsigned int SourceX2, unsigned int SourceX3,
                                unsigned int SourceX4, CMDPACTION* action, int cost);

    virtual void ComputeHeuristicValues();
};


#endif
