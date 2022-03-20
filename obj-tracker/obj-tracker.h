/**
 *  Andrew Capatina 
 *  Ad-hoc Defense
 * 
 *  Object tracking header file.
 * 
 * */
#include <vector>
using namespace std;

#define NUM_SEC_NO_DETECT 5

enum cam_states 
{
    IDLE,
    TRACKING,
    SAVING,
    SENDING
};

// Function protypes
// Not all functions included for lib purposes. 
// Prefer to keep the libs in the cpp file. 
// Only function to be used by other files for now 
// is the object_tracker() anyways.
void sig_handler(int signo);
void * object_tracker(void * threadID);

double hungarian_solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment);
void assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);

