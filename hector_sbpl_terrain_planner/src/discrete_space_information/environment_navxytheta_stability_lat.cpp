//Implementation for new environment, analogous to https://github.com/sbpl/sbpl/tree/master/src/discrete_space_information

#include <hector_sbpl_terrain_planner/discrete_space_information/environment_navxytheta_stability_lat.h>



//#include <sbpl/discrete_space_information/environment_XXX.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

using namespace std;

//extern clock_t time3_addallout;
//extern clock_t time_gethash;
//extern clock_t time_createhash;

//function prototypes

//-------------------problem specific and local functions---------------------

static unsigned int inthash(unsigned int key)
{
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4);
    key ^= (key >> 9);
    key += (key << 10);
    key ^= (key >> 2);
    key += (key << 7);
    key ^= (key >> 12);
    return key;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X)
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAVXYTHETASTAB::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4)
{
    return inthash((inthash(X1) + (inthash(X2) << 1) + (inthash(X3) << 2) + (inthash(X4) << 3))) &
           (EnvNAVXYTHETASTAB.HashTableSize - 1);
}

void EnvironmentNAVXYTHETASTAB::PrintHashTableHist()
{
    int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

    for (int j = 0; j < (int)EnvNAVXYTHETASTAB.HashTableSize; j++) {
        if ((int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[j].size() == 0)
            s0++;
        else if ((int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[j].size() < 50)
            s1++;
        else if ((int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[j].size() < 100)
            s50++;
        else if ((int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[j].size() < 200)
            s100++;
        else if ((int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[j].size() < 300)
            s200++;
        else if ((int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[j].size() < 400)
            s300++;
        else
            slarge++;
    }
    SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n", s0, s1, s50, s100,
                s200, s300, slarge);
}

void EnvironmentNAVXYTHETASTAB::ReadConfiguration(FILE* fCfg)
{
    //read in the configuration of environment and initialize EnvCfg structure
}

void EnvironmentNAVXYTHETASTAB::InitializeEnvConfig()
{
    //aditional to configuration file initialization of EnvCfg if necessary
}

EnvNAVXYTHETASTABHashEntry_t* EnvironmentNAVXYTHETASTAB::GetHashEntry(unsigned int X1, unsigned int X2, unsigned int X3, unsigned int X4)
{
    //clock_t currenttime = clock();

    int binid = GETHASHBIN(X1, X2, X3, X4);

#if DEBUG
    if ((int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid].size() > 500)
    {
        SBPL_PRINTF("WARNING: Hash table has a bin %d (X1=%d X2=%d X3=%d X4=%d) of size %d\n",
                    binid, X1, X2, X3, X4, (int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid].size());

        PrintHashTableHist();
    }
#endif

    //iterate over the states in the bin and select the perfect match
    for (int ind = 0; ind < (int)EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid].size(); ind++) {
        if (EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid][ind]->X1 == X1 &&
            EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid][ind]->X2 == X2 &&
            EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid][ind]->X3 == X3 &&
            EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid][ind]->X4 == X4)
        {
            //time_gethash += clock()-currenttime;
            return EnvNAVXYTHETASTAB.Coord2StateIDHashTable[binid][ind];
        }
    }

    //time_gethash += clock()-currenttime;

    return NULL;
}

EnvNAVXYTHETASTABHashEntry_t* EnvironmentNAVXYTHETASTAB::CreateNewHashEntry(unsigned int X1, unsigned int X2, unsigned int X3,
                                                      unsigned int X4)
{
    int i;

    //clock_t currenttime = clock();

    EnvNAVXYTHETASTABHashEntry_t* HashEntry = new EnvNAVXYTHETASTABHashEntry_t;

    HashEntry->X1 = X1;
    HashEntry->X2 = X2;
    HashEntry->X3 = X3;
    HashEntry->X4 = X4;

    HashEntry->stateID = EnvNAVXYTHETASTAB.StateID2CoordTable.size();

    //insert into the tables
    EnvNAVXYTHETASTAB.StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->X1, HashEntry->X2, HashEntry->X3, HashEntry->X4);

    //insert the entry into the bin
    EnvNAVXYTHETASTAB.Coord2StateIDHashTable[i].push_back(HashEntry);

    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }

    if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

    //time_createhash += clock()-currenttime;

    return HashEntry;
}

void EnvironmentNAVXYTHETASTAB::CreateStartandGoalStates()
{
    EnvNAVXYTHETASTABHashEntry_t* HashEntry;

    //create start state
    unsigned int X1 = 0;
    unsigned int X2 = 0;
    unsigned int X3 = 0;
    unsigned int X4 = 0;
    HashEntry = CreateNewHashEntry(X1, X2, X3, X4);
    EnvNAVXYTHETASTAB.startstateid = HashEntry->stateID;

    //create goal state
    X1 = X2 = X3 = X4 = 1;
    HashEntry = CreateNewHashEntry(X1, X2, X3, X4);
    EnvNAVXYTHETASTAB.goalstateid = HashEntry->stateID;
}

void EnvironmentNAVXYTHETASTAB::InitializeEnvironment()
{

    //initialize the map from Coord to StateID
    EnvNAVXYTHETASTAB.HashTableSize = 32 * 1024; //should be power of two
    EnvNAVXYTHETASTAB.Coord2StateIDHashTable = new vector<EnvNAVXYTHETASTABHashEntry_t*> [EnvNAVXYTHETASTAB.HashTableSize];

    //initialize the map from StateID to Coord
    EnvNAVXYTHETASTAB.StateID2CoordTable.clear();

    //create start and goal states
    CreateStartandGoalStates();
}

void EnvironmentNAVXYTHETASTAB::AddAllOutcomes(unsigned int SourceX1, unsigned int SourceX2, unsigned int SourceX3,
                                    unsigned int SourceX4, CMDPACTION* action, int cost)
{
    EnvNAVXYTHETASTABHashEntry_t* OutHashEntry;
    int i;
    float CumProb = 0.0;

    //iterate over outcomes
    for (i = 0; i < 2; i++) {
        unsigned int newX1 = SourceX1 + i;
        unsigned int newX2 = SourceX2 + i;
        unsigned int newX3 = SourceX3 + i;
        unsigned int newX4 = SourceX4 + i;

        //add the outcome
        if ((OutHashEntry = GetHashEntry(newX1, newX2, newX3, newX4)) == NULL) {
            //have to create a new entry
            OutHashEntry = CreateNewHashEntry(newX1, newX2, newX3, newX4);
        }
        float Prob = 0.5; //probability of the outcome
        action->AddOutcome(OutHashEntry->stateID, cost, Prob);
        CumProb += Prob;

    } //while

    if (CumProb != 1.0) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: prob. of all action outcomes=%f\n", CumProb);
        throw new SBPL_Exception();
    }
}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAVXYTHETASTAB::ComputeHeuristicValues()
{
    //whatever necessary pre-computation of heuristic values is done here
    SBPL_PRINTF("Precomputing heuristics\n");

    SBPL_PRINTF("done\n");
}

//-----------interface with outside functions-----------------------------------

bool EnvironmentNAVXYTHETASTAB::InitializeEnv(const char* sEnvFile)
{
    FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    ReadConfiguration(fCfg);
    fclose(fCfg);

    //Initialize other parameters of the environment
    InitializeEnvConfig();

    //initialize Environment
    InitializeEnvironment();

    //pre-compute heuristics
    ComputeHeuristicValues();

    return true;
}

bool EnvironmentNAVXYTHETASTAB::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = EnvNAVXYTHETASTAB.goalstateid;
    MDPCfg->startstateid = EnvNAVXYTHETASTAB.startstateid;

    return true;
}

int EnvironmentNAVXYTHETASTAB::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)EnvNAVXYTHETASTAB.StateID2CoordTable.size() ||
       ToStateID >= (int)EnvNAVXYTHETASTAB.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it is used in the planner

    SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB.. function: FromToHeuristic is undefined\n");
    throw new SBPL_Exception();

    return 0;
}

int EnvironmentNAVXYTHETASTAB::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvNAVXYTHETASTAB.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic forward search would use it)

    SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB..function: GetGoalHeuristic is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentNAVXYTHETASTAB::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if (stateID >= (int)EnvNAVXYTHETASTAB.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: stateID illegal\n");
        throw new SBPL_Exception();
    }
#endif

    //define this function if it used in the planner (heuristic backward search would use it)

    SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB.. function: GetStartHeuristic is undefined\n");
    throw new SBPL_Exception();

    return 0;
}

void EnvironmentNAVXYTHETASTAB::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

#if DEBUG
    if (state->StateID >= (int)EnvNAVXYTHETASTAB.StateID2CoordTable.size()) {
        SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: stateID illegal\n");
        throw new SBPL_Exception();
    }

    if ((int)state->Actions.size() != 0) {
        SBPL_ERROR("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
        throw new SBPL_Exception();
    }
#endif

    //if it is goal then no successors
    if (state->StateID == EnvNAVXYTHETASTAB.goalstateid) return;

    //get values for the state
    EnvNAVXYTHETASTABHashEntry_t* HashEntry = EnvNAVXYTHETASTAB.StateID2CoordTable[state->StateID];

    //iterate through the actions for the state
    for (int aind = 0; aind < XXX_MAXACTIONSWIDTH; aind++) {
        int cost = 1;

        //Add Action
        CMDPACTION* action = state->AddAction(aind);

        //clock_t currenttime = clock();
        //add all the outcomes to the action
        AddAllOutcomes(HashEntry->X1, HashEntry->X2, HashEntry->X3, HashEntry->X4, action, cost);

        //you can break if the number of actual actions is smaller than the maximum possible

        //time3_addallout += clock()-currenttime;
    }
}

void EnvironmentNAVXYTHETASTAB::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentNAVXYTHETASTAB::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: GetSuccs is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentNAVXYTHETASTAB::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
    SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: GetPreds is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentNAVXYTHETASTAB::SizeofCreatedEnv()
{
    return (int)EnvNAVXYTHETASTAB.StateID2CoordTable.size();
}

void EnvironmentNAVXYTHETASTAB::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
    if(stateID >= (int)EnvNAVXYTHETASTAB.StateID2CoordTable.size())
    {
        SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: stateID illegal (2)\n");
        throw new SBPL_Exception();
    }
#endif

    if (fOut == NULL) fOut = stdout;

    EnvNAVXYTHETASTABHashEntry_t* HashEntry = EnvNAVXYTHETASTAB.StateID2CoordTable[stateID];

    if (stateID == EnvNAVXYTHETASTAB.goalstateid) {
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    SBPL_FPRINTF(fOut, "X1=%d X2=%d X3=%d X4=%d\n", HashEntry->X1, HashEntry->X2, HashEntry->X3, HashEntry->X4);
}

void EnvironmentNAVXYTHETASTAB::PrintEnv_Config(FILE* fOut)
{
    //implement this if the planner needs to print out EnvNAVXYTHETASTAB. configuration

    SBPL_ERROR("ERROR in EnvNAVXYTHETASTAB... function: PrintEnv_Config is undefined\n");
    throw new SBPL_Exception();
}


