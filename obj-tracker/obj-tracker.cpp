/*
 * Copyright (c) 2021, Andrew Capatina
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
/**
 *  Andrew Capatina
 *  6/28/2021
 * 
 *  Ad-hoc Defense.
 * 
 *  Description:
 *      Code integrating camera interface, servos, and nueral network.
 *  Uses a PID loop to track an object. 
 * 
 **/

#include <opencv4/opencv2/opencv.hpp>

#include <cuda.h>
#include <jetson-utils/cudaRGB.h>


#include <jetson-inference/detectNet.h>
#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>

#include "obj-tracker.h"
#include "servo_driver/servo_driver.h"
#include "nrf24/nrf24.h"

#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <algorithm>


#include <stdlib.h>
#include <cfloat>
#include <cmath>

double hungarian_solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment);
void assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns);
void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
/*
 * FSM related initializations.
 * */
enum state_codes {init, idle, tracking, saving, sending, endd};
enum return_codes {ok, repeat, err};
enum return_codes init_state(void);
enum return_codes idle_state(void);
enum return_codes tracking_state(cv::Mat * image_array);
enum return_codes saving_state(cv::Mat * image_array);
enum return_codes sending_state(void);
enum return_codes end_state(void);
enum state_codes lookup_next_state(enum state_codes sc, enum return_codes rc);

//typedef enum return_codes (*function_ptr)(void);
//const int NUM_FUNC_STATES = 6;
//function_ptr func_table[NUM_FUNC_STATES] = {(function_ptr)init_state, (function_ptr)idle_state, (function_ptr)tracking_state, (function_ptr)saving_state, (function_ptr)sending_state, (function_ptr)end_state};

struct transition
{
	enum state_codes current_state;
	enum return_codes return_code;
	enum state_codes next_state;
};

struct transition lookup_table[][5] = 
{
	{
		{init, ok, idle},
		{init, repeat, init},
		{init, err, endd}
	},
	{
		{idle, ok, tracking},
		{idle, repeat, idle},
		{idle, err, endd}
	},
	{
		{tracking, ok, saving},
		{tracking, repeat, tracking},
		{tracking, err, endd}
	},
	{
		{saving, ok, sending},
		{saving, repeat, saving},
		{saving, err, endd}
	},
	{
		{sending, ok, idle},
		{sending, repeat, sending},
		{sending, err, endd}
	},
};

/*
 * Global variables. 
 * */
bool signal_received = false;
char camera_name[20] = "camera_1.0";
// Interface initialization variables.
int i2c_fd;
videoOutput* output = NULL;
detectNet* net;
cv::VideoCapture * cap;
// Image variables. 
cv::Mat img;  
// Convert MAT image type to uchar3.
uchar3 * img_buffer;
cv::Mat img_bgr;

// Return code variables.
cudaError_t cuda_rtn;
int i2c_rtn;

// resolution = width X height.
// width = columns.
// height = rows.
int capture_width = 1280 ;
int capture_height = 720 ;
int display_width = 1280 ;
int display_height = 720 ;
int framerate = 30 ;
int flip_method = 2 ;
bool img_grabbed = false;


// PID loop initializations.
clock_t ticks_start, ticks_end;     // clock ticks
float iteration_time = 0;           // iteration time.

float error_prior_x = 0;
float integral_prior_x = 0;
float derivative_prior_x = 0;

float error_prior_y = 0;
float integral_prior_y = 0;
float derivative_prior_y = 0;

float kp = 0.03;     // PID loop tuning parameters.
float ki = 0.025;
float kd = 0.00;
int bias = 0;

uint32_t cam_center_y = capture_height / 2;
uint32_t cam_center_x = capture_width / 2;


// NRF24 SPI decleration..
char rising[7] = GPIO_EDGE_RISE;
const char dev[32] = "/dev/spidev0.0";
uint8_t mode = 0x00;	// SPI_MODE_0.
uint8_t bits = 8;
uint32_t speed = 1000000;
int lsb_setting = 0;
int nrf_spi_fd;

// pan and tilt decleration
__uint8_t pan = 90;
__uint8_t tilt = 90;
__uint8_t pan_set = pan;
__uint8_t tilt_set = tilt;

bool detection_found = false;   // If a detection is found. 
bool record_done = false;       // Indicates if recording is finished.
float record_seconds = 5.0;     // amount of seconds to record.
float cooldown_time = 0.0;      // Time to cooldown after recording.
int frame_idx = 0;              // Current frame index, used for saving images.

const uint32_t overlay_flags = detectNet::OverlayFlagsFromStr("box,labels,conf");

float time_no_detect = 0.0;
bool lock_no_detect = false;

vector <double> prev_x1;
vector <double> prev_x2;
vector <double> prev_y1;
vector <double> prev_y2;
vector <double> prev_area;

double x1_p;
double x2_p;
double y1_p;
double y2_p;
double area;

vector<vector<double>> iou_scores;

detectNet::Detection* detections;
int number_frames = record_seconds * framerate;        // Number of frames in recording time.
int num_detections;



void sig_handler(int signo)
{
    if(signo == SIGINT)
    {
        LogVerbose("received SIGINT\n");
        signal_received=true;
    }
}

string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{

    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + 
            ", height=(int)" + to_string(capture_height) +
            ", format=(string)NV12" + ", framerate=(fraction)" + to_string(framerate) + "/1 " +
	    "! queue " + 
            "! nvvidconv flip-method=" + to_string(flip_method) +
            " ! video/x-raw, format=(string)BGRx, width=(int)" + to_string(display_width) + ", height=(int)" + to_string(display_height) +
            " ! videoconvert" +
            " ! appsink drop=1";
}

/**
 * 
 * Function to deallocate all hardware interfaces and memory.
 * */
int free_resources(int i2c_fd, uchar3 * img_buffer, detectNet * net) 
{
    i2c_close(i2c_fd);
    CUDA_FREE(img_buffer);
    SAFE_DELETE(net);
    gpio_unexport(GPIO_CE);
    gpio_unexport(GPIO_IRQ);

    return 0;
}

void * object_tracker(void * threadID) 
{
    cv::Mat image_array[number_frames];     // Array of images containing the video.
    enum state_codes curr_state = init;
    enum return_codes ret_code = ok;

    ret_code = init_state();
    //enum return_codes (*func)(void) = NULL;
    while(1)
    {
        // Get start clock ticks.
        ticks_start = clock();

        img_grabbed = cap->read(img_bgr);

        if(img_grabbed == true)
        {
            cv::cvtColor(img_bgr, img, cv::COLOR_BGR2RGB);

            cuda_rtn = cudaMemcpy2D((void*) img_buffer, capture_width*sizeof(uchar3), (void*) img.data, (size_t) img.step,
                    capture_width*sizeof(uchar3), capture_height, cudaMemcpyHostToDevice);
        
            if(cuda_rtn != 0)
            {
                printf("cudaMemcpy2D() error: %i\n", cuda_rtn);
                break;
            }

            	    
            // image parameter is uchar3.
            num_detections = net->Detect(img_buffer, (uint32_t) capture_width,(uint32_t) capture_height, &detections, overlay_flags);

	    if(signal_received)
	    {
		    curr_state = endd;
	    }

	    switch(curr_state)
	    {
		case idle:
			ret_code = idle_state();
			break;
		case tracking:
			ret_code = tracking_state(image_array);
			break;
		case saving:
			ret_code = saving_state(image_array);
			break;
		case sending:
			ret_code = sending_state();
			break;
		case endd:
			ret_code = end_state();
			break;
		default:
			break;
	    }

	    if(curr_state == endd)
	    {
		    break;
	    }

	    curr_state = lookup_next_state(curr_state, ret_code);


/*	    
            if(output != NULL)
            {
                output->Render(img_buffer, (uint32_t) display_width, (uint32_t) display_height);

                //Update the status bar
                char str[256];
                sprintf(str, "TensorRT %i.%i.%i | %s | Network %.0f FPS", NV_TENSORRT_MAJOR, NV_TENSORRT_MINOR, NV_TENSORRT_PATCH, precisionTypeToStr(net->GetPrecision()), net->GetNetworkFPS());
                output->SetStatus(str);

                if(!output->IsStreaming())
                    signal_received = true;

            }
*/

            iteration_time = (float) (clock() - ticks_start) / CLOCKS_PER_SEC;
        }
        

        //net->PrintProfilerTimes();
    }

    pthread_exit(0);

}

//********************************************************//
// A single function wrapper for solving assignment problem.
//********************************************************//
double hungarian_solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment)
{
	unsigned int nRows = DistMatrix.size();
	unsigned int nCols = DistMatrix[0].size();

	double *distMatrixIn = new double[nRows * nCols];
	int *assignment = new int[nRows];
	double cost = 0.0;

	// Fill in the distMatrixIn. Mind the index is "i + nRows * j".
	// Here the cost matrix of size MxN is defined as a double precision array of N*M elements. 
	// In the solving functions matrices are seen to be saved MATLAB-internally in row-order.
	// (i.e. the matrix [1 2; 3 4] will be stored as a vector [1 3 2 4], NOT [1 2 3 4]).
	for (unsigned int i = 0; i < nRows; i++)
		for (unsigned int j = 0; j < nCols; j++)
			distMatrixIn[i + nRows * j] = DistMatrix[i][j];
	
	// call solving function
	assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

	Assignment.clear();
	for (unsigned int r = 0; r < nRows; r++)
		Assignment.push_back(assignment[r]);

	delete[] distMatrixIn;
	delete[] assignment;
	return cost;
}


//********************************************************//
// Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
//********************************************************//
void assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
	double *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
	bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
	int nOfElements, minDim, row, col;

	/* initialization */
	*cost = 0;
	for (row = 0; row<nOfRows; row++)
		assignment[row] = -1;

	/* generate working copy of distance Matrix */
	/* check if all matrix elements are positive */
	nOfElements = nOfRows * nOfColumns;
	distMatrix = (double *)malloc(nOfElements * sizeof(double));
	distMatrixEnd = distMatrix + nOfElements;

	for (row = 0; row<nOfElements; row++)
	{
		value = distMatrixIn[row];
		if (value < 0)
			cerr << "All matrix elements have to be non-negative." << endl;
		distMatrix[row] = value;
	}


	/* memory allocation */
	coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool));
	coveredRows = (bool *)calloc(nOfRows, sizeof(bool));
	starMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool)); /* used in step4 */

	/* preliminary steps */
	if (nOfRows <= nOfColumns)
	{
		minDim = nOfRows;

		for (row = 0; row<nOfRows; row++)
		{
			/* find the smallest element in the row */
			distMatrixTemp = distMatrix + row;
			minValue = *distMatrixTemp;
			distMatrixTemp += nOfRows;
			while (distMatrixTemp < distMatrixEnd)
			{
				value = *distMatrixTemp;
				if (value < minValue)
					minValue = value;
				distMatrixTemp += nOfRows;
			}

			/* subtract the smallest element from each element of the row */
			distMatrixTemp = distMatrix + row;
			while (distMatrixTemp < distMatrixEnd)
			{
				*distMatrixTemp -= minValue;
				distMatrixTemp += nOfRows;
			}
		}

		/* Steps 1 and 2a */
		for (row = 0; row<nOfRows; row++)
			for (col = 0; col<nOfColumns; col++)
				if (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
					if (!coveredColumns[col])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true;
						break;
					}
	}
	else /* if(nOfRows > nOfColumns) */
	{
		minDim = nOfColumns;

		for (col = 0; col<nOfColumns; col++)
		{
			/* find the smallest element in the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			columnEnd = distMatrixTemp + nOfRows;

			minValue = *distMatrixTemp++;
			while (distMatrixTemp < columnEnd)
			{
				value = *distMatrixTemp++;
				if (value < minValue)
					minValue = value;
			}

			/* subtract the smallest element from each element of the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			while (distMatrixTemp < columnEnd)
				*distMatrixTemp++ -= minValue;
		}

		/* Steps 1 and 2a */
		for (col = 0; col<nOfColumns; col++)
			for (row = 0; row<nOfRows; row++)
				if (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
					if (!coveredRows[row])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true;
						coveredRows[row] = true;
						break;
					}
		for (row = 0; row<nOfRows; row++)
			coveredRows[row] = false;

	}

	/* move to step 2b */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

	/* compute cost and remove invalid assignments */
	computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);

	/* free allocated memory */
	free(distMatrix);
	free(coveredColumns);
	free(coveredRows);
	free(starMatrix);
	free(primeMatrix);
	free(newStarMatrix);

	return;
}

/********************************************************/
void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
{
	int row, col;

	for (row = 0; row<nOfRows; row++)
		for (col = 0; col<nOfColumns; col++)
			if (starMatrix[row + nOfRows*col])
			{
#ifdef ONE_INDEXING
				assignment[row] = col + 1; /* MATLAB-Indexing */
#else
				assignment[row] = col;
#endif
				break;
			}
}

/********************************************************/
void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows)
{
	int row, col;

	for (row = 0; row<nOfRows; row++)
	{
		col = assignment[row];
		if (col >= 0)
			*cost += distMatrix[row + nOfRows*col];
	}
}

/********************************************************/
void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool *starMatrixTemp, *columnEnd;
	int col;

	/* cover every column containing a starred zero */
	for (col = 0; col<nOfColumns; col++)
	{
		starMatrixTemp = starMatrix + nOfRows*col;
		columnEnd = starMatrixTemp + nOfRows;
		while (starMatrixTemp < columnEnd){
			if (*starMatrixTemp++)
			{
				coveredColumns[col] = true;
				break;
			}
		}
	}

	/* move to step 3 */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	int col, nOfCoveredColumns;

	/* count covered columns */
	nOfCoveredColumns = 0;
	for (col = 0; col<nOfColumns; col++)
		if (coveredColumns[col])
			nOfCoveredColumns++;

	if (nOfCoveredColumns == minDim)
	{
		/* algorithm finished */
		buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
	}
	else
	{
		/* move to step 3 */
		step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}

}

/********************************************************/
void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool zerosFound;
	int row, col, starCol;

	zerosFound = true;
	while (zerosFound)
	{
		zerosFound = false;
		for (col = 0; col<nOfColumns; col++)
			if (!coveredColumns[col])
				for (row = 0; row<nOfRows; row++)
					if ((!coveredRows[row]) && (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON))
					{
						/* prime zero */
						primeMatrix[row + nOfRows*col] = true;

						/* find starred zero in current row */
						for (starCol = 0; starCol<nOfColumns; starCol++)
							if (starMatrix[row + nOfRows*starCol])
								break;

						if (starCol == nOfColumns) /* no starred zero found */
						{
							/* move to step 4 */
							step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
							return;
						}
						else
						{
							coveredRows[row] = true;
							coveredColumns[starCol] = false;
							zerosFound = true;
							break;
						}
					}
	}

	/* move to step 5 */
	step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
	int n, starRow, starCol, primeRow, primeCol;
	int nOfElements = nOfRows*nOfColumns;

	/* generate temporary copy of starMatrix */
	for (n = 0; n<nOfElements; n++)
		newStarMatrix[n] = starMatrix[n];

	/* star current zero */
	newStarMatrix[row + nOfRows*col] = true;

	/* find starred zero in current column */
	starCol = col;
	for (starRow = 0; starRow<nOfRows; starRow++)
		if (starMatrix[starRow + nOfRows*starCol])
			break;

	while (starRow<nOfRows)
	{
		/* unstar the starred zero */
		newStarMatrix[starRow + nOfRows*starCol] = false;

		/* find primed zero in current row */
		primeRow = starRow;
		for (primeCol = 0; primeCol<nOfColumns; primeCol++)
			if (primeMatrix[primeRow + nOfRows*primeCol])
				break;

		/* star the primed zero */
		newStarMatrix[primeRow + nOfRows*primeCol] = true;

		/* find starred zero in current column */
		starCol = primeCol;
		for (starRow = 0; starRow<nOfRows; starRow++)
			if (starMatrix[starRow + nOfRows*starCol])
				break;
	}

	/* use temporary copy as new starMatrix */
	/* delete all primes, uncover all rows */
	for (n = 0; n<nOfElements; n++)
	{
		primeMatrix[n] = false;
		starMatrix[n] = newStarMatrix[n];
	}
	for (n = 0; n<nOfRows; n++)
		coveredRows[n] = false;

	/* move to step 2a */
	step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

/********************************************************/
void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	double h, value;
	int row, col;

	/* find smallest uncovered element h */
	h = DBL_MAX;
	for (row = 0; row<nOfRows; row++)
		if (!coveredRows[row])
			for (col = 0; col<nOfColumns; col++)
				if (!coveredColumns[col])
				{
					value = distMatrix[row + nOfRows*col];
					if (value < h)
						h = value;
				}

	/* add h to each covered row */
	for (row = 0; row<nOfRows; row++)
		if (coveredRows[row])
			for (col = 0; col<nOfColumns; col++)
				distMatrix[row + nOfRows*col] += h;

	/* subtract h from each uncovered column */
	for (col = 0; col<nOfColumns; col++)
		if (!coveredColumns[col])
			for (row = 0; row<nOfRows; row++)
				distMatrix[row + nOfRows*col] -= h;

	/* move to step 3 */
	step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

enum return_codes init_state(void)
{
    if(signal(SIGINT, sig_handler) == SIG_ERR)
    {
        LogError("cant catch SIGINT\n");
    }

    // total size a function of width, bytes, and height.
    cuda_rtn = cudaMalloc((void**) &img_buffer, (size_t) capture_width * sizeof(uchar3) * capture_height);
    if(cuda_rtn != 0)
    {
        printf("cudaMalloc() error: %i\n", cuda_rtn);
	return err;
    }

    gpio_export(GPIO_CE);
    gpio_set_dir(GPIO_CE, GPIO_DIR_OUTPUT);	// Set GPIO_CE as output.

    gpio_export(GPIO_IRQ);
    gpio_set_dir(GPIO_IRQ, GPIO_DIR_INPUT);	// Set GPIO_IRQ as input.
    gpio_set_edge(GPIO_IRQ, rising);
    gpio_set_active_edge(GPIO_IRQ);	// Set to active low.

    nrf_spi_fd = spi_init(dev, mode, bits, speed, lsb_setting);

    if(nrf_spi_fd < 0)
    {
	printf("Failed to initialize SPI: %i \n", nrf_spi_fd);
	return err;
    }

    char addr;
    char msg[5];

    /**
     *	NOTE: For auto-ack transmit and receive 
     *	address must be equivalent.
     * */
    msg[0] = 'R'; msg[1] = 'x'; msg[2] = 'A';
    msg[3] = 'A'; msg[4] = 'A';

    nrf_set_rx_address(nrf_spi_fd, msg, 0);

    nrf_set_tx_address(nrf_spi_fd, msg);

    nrf_tx_init(nrf_spi_fd);

    /* 
        I2C & slave initialization.
    */
    i2c_fd = i2c_init(adapter_nr_global, addr_global);

    if(i2c_fd < 0)
    {
        printf("i2c_init() failed.\n");
        return err;
    }

    i2c_rtn = PCA9685_init(i2c_fd);

    if(i2c_rtn < 0)
    {
        printf("PCA9685_init() failed.\n");
	return err;
    }

    if(PCA9685_set_servo_degree(i2c_fd, SERVO_DOWN_CH, pan) < 0)
    {
        printf("PCA9685_set_servo_degree() failed.\n");
	return err;
    }

    if(PCA9685_set_servo_degree(i2c_fd, SERVO_UP_CH, tilt) < 0)
    {
        printf("PCA9685_set_servo_degree() failed.\n");
	return err;
    }

    /*
        Video and object detection network initialization.
    */
/*
    output = videoOutput::Create("display://0");

    if(!output)
    {
        LogError("detectnet: failed to create output stream\n");
        free_resources(i2c_fd,img_buffer,net,output);
    	pthread_exit((void *) 1);
    }
*/

    net = detectNet::Create(NULL,"/home/cap/Projects/object_detection/jetson-inference/python/training/detection/ssd/models/person/ssd-mobilenet.onnx",
                            0.0f,
                            "/home/cap/Projects/object_detection/jetson-inference/python/training/detection/ssd/models/person/labels.txt",
                            0.5f, "input_0","scores", "boxes",DEFAULT_MAX_BATCH_SIZE,
                            TYPE_FASTEST,DEVICE_GPU, true);

    if(!net)
    {
        LogError("detectnet: Failed to load detectNet model\n");
	return err;
    }

    /*
        Video input stream initiazation.
    */ 

    string pipeline = gstreamer_pipeline(capture_width,
        capture_height,
        display_width,
        display_height,
        framerate,
        flip_method);
    cout << "Using pipeline: \n\t" << pipeline << "\n";

    cap = new cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
    if(!cap->isOpened()) {
        std::cout<<"Failed to open camera."<<std::endl;
    	return err;    
    }


	return ok;
}
enum return_codes idle_state(void)
{

    // Logic to reset the servos after a detection is no longer found. 
    if(lock_no_detect == true)
    {
	time_no_detect = time_no_detect + iteration_time;

	if(time_no_detect > (float) NUM_SEC_NO_DETECT)
	{
	    lock_no_detect = false;
	    //time_no_detect = 0.0;
	    
	    i2c_rtn = PCA9685_set_servo_degree(i2c_fd, SERVO_DOWN_CH, pan_set);

	    if(i2c_rtn < 0)
	    {
		printf("PCA9685_set_servo_degree() DOWN_CHANNEL failed.\n");

		return err;
	    }

	    i2c_rtn = PCA9685_set_servo_degree(i2c_fd, SERVO_UP_CH, tilt_set);

	    if(i2c_rtn < 0)
	    {
		printf("PCA9685_set_servo_degree() UP_CHANNEL failed.\n");

		return err;
	    }

	    pan = pan_set;      // Reset the pan and tilt variables.
	    tilt = tilt_set;
	}

    }

    if(num_detections > 0)
    {
	lock_no_detect = true;
	time_no_detect = 0.0;
	return ok; // transition to next state when detection found.
    }

	return repeat;
}
enum return_codes tracking_state(cv::Mat * image_array)
{
    //cout << "NUM_DETECTIONS: " << num_detections << endl;

    // Implement tracking here

    // Calculate IOU scores.
    for(int i = 0; i < num_detections; ++i)
    {
	detection_found = true; // TODO: Remove this 


	// Assign bounding box coordinates.
	x1_p = (double) detections[i].Left;
	y1_p = (double) detections[i].Bottom;
	x2_p = (double) detections[i].Right;
	y2_p = (double) detections[i].Top;
	area = (double) detections[i].Area();


	// Check if there are previous bounding boxes.
	if(prev_area.size() > 0)
	{
		//cout << "SIZE: " << prev_area.size() << endl;
		iou_scores.push_back(vector<double>(prev_area.size(), 0));

		for(int j = 0; j < prev_area.size(); ++j)
		{
			// Determine coordinates of intersection rectangle.
			double x_left = max(x1_p, prev_x1[j]);
			double y_top = max(y2_p, prev_y2[j]);
			double y_bottom = min(y1_p, prev_y1[j]);
			double x_right = min(x2_p, prev_x2[j]);

			if((x_right < x_left) || (y_bottom < y_top))
			{
				iou_scores[i][j] = 0.0;
				continue;
			}
			// The intersection is always axis-aligned.
			double intersectionArea = (x_right - x_left) * (y_bottom - y_top);
			// Compute the Intersection Over Union.
			double iou = intersectionArea / (double) (prev_area[j] + area - intersectionArea);

			iou_scores[i][j] = iou;
		}
	}
     }	    

    if(iou_scores.size() > 0)
    {
	    vector<int> assignment;

	    hungarian_solve(iou_scores, assignment);

	    /*
	    cout <<"ASSIGNMENT VECTOR: " << endl;
	    for(int i = 0; i < assignment.size(); ++i)
	    {
		    cout << assignment[i] << endl;
	    }
	    */
    }


	prev_x1.clear();	
	prev_y1.clear();	
	prev_x2.clear();	
	prev_y2.clear();	
	prev_area.clear();	
	// Save the bbox and area before the next 
	// iteration.
	for(int i = 0; i < num_detections; ++i)
	{
	// Assign bounding box coordinates.
	prev_x1.push_back(detections[i].Left);
	prev_y1.push_back(detections[i].Bottom);
	prev_x2.push_back(detections[i].Right);
	prev_y2.push_back(detections[i].Top);
	prev_area.push_back(detections[i].Area());


	/*
	cout << "IOU_SCORE_MATRIX" << endl;
	for(int i = 0; i < iou_scores.size(); ++i)
	{
		for(int j = 0; j < iou_scores[i].size(); ++j)
		{
			cout << iou_scores[i][j] << " ";
		}	
		cout << endl;
	}
	*/

	iou_scores.clear();
	}



    for(int i = 0; i < num_detections; ++i)
    {
	if(detections[i].ClassID == 5)
	{
	    if(detection_found == false)
	    {
		detection_found = true;
	    }

	    float x_loc, y_loc;
	    detections[i].Center(&x_loc, &y_loc);

	    // Get the error from the center point.
	    float x_error = x_loc - cam_center_x;
	    float y_error = y_loc - cam_center_y;

	    if(abs(x_error) > 50)
	    {
		// Get the integral value.
		float integral_x = ((ki*iteration_time)/2) * (x_error+error_prior_x) + integral_prior_x;
		
		// Get the derivative value.
		float derivative_x = -(2.0 * kd *(x_error - error_prior_x)
				    + (2.0 * 0.5 - iteration_time) * derivative_prior_x)
				    / (2.0 * 0.5 + iteration_time);

		// Get the output pan/tilt step values.
		int pan_step = kp*x_error + integral_x + derivative_x + bias;

		if(pan_step < -180)
		{
		    pan_step = -180;
		}
		else if(pan_step > 180)
		{
		    pan_step = 180;
		}

		pan = pan + pan_step;
		i2c_rtn = PCA9685_set_servo_degree(i2c_fd, SERVO_DOWN_CH, pan);

		if(i2c_rtn < 0)
		{
		    printf("PCA9685_set_servo_degree() DOWN_CHANNEL failed.\n");
		    return ok;
		}

		// Feedback variables.
		error_prior_x = x_error;
		integral_prior_x = integral_x;

		printf("pan vals \n");
		printf("iteration_time: %f \n", iteration_time);
		printf("x_error: %f \n", x_error);
		printf("integral_x: %f \n", integral_x);
		printf("derivative_x: %f \n", derivative_x);
		printf("pan_step: %i \n", pan_step);
		printf("pan: %i \n", pan);
	    }

	    if(abs(y_error) > 50)
	    {
		// Get the integral value.
		float integral_y = ((ki*iteration_time)/2) * (y_error+error_prior_y) + integral_prior_y;
		
		// Get the derivative value.
		float derivative_y = -(2.0 * kd *(y_error - error_prior_y)
				    + (2.0 * 0.5 - iteration_time) * derivative_prior_y)
				    / (2.0 * 0.5 + iteration_time);

		// Get the output pan/tilt step values.
		int tilt_step = kp*x_error + integral_y + derivative_y + bias;

		if(tilt_step < -180)
		{
		    tilt_step = -180;
		}
		else if(tilt_step > 180)
		{
		    tilt_step = 180;
		}

		tilt = tilt + tilt_step;
		i2c_rtn = PCA9685_set_servo_degree(i2c_fd, SERVO_UP_CH, tilt);

		if(i2c_rtn < 0)
		{
		    printf("PCA9685_set_servo_degree() UP_CHANNEL failed.\n");

		    return ok;	// TODO: what to do here?
		}

		// Feedback variables.
		error_prior_y = y_error;
		integral_prior_y = integral_y;

		printf("tilt vals \n");
		printf("iteration_time: %f \n", iteration_time);
		printf("y_error: %f \n", y_error);
		printf("integral_y: %f \n", integral_y);
		printf("derivative_y: %f \n", derivative_y);
		printf("tilt_step: %i \n", tilt_step);
		printf("tilt: %i \n", tilt);
	    }
	}
    }

    if(detection_found == true)
    {
	//printf("Saved image! %i\n", frame_idx);
	image_array[frame_idx] = img_bgr;
	if(frame_idx == number_frames - 1)
	{
	    frame_idx = 0;
	    return ok;	// transition to next state once all frames saved.
	}
	frame_idx += 1;
	img_bgr = cv::Mat();    // Reset the image variable so the pixels are refreshed.
    }

	return repeat;
}
enum return_codes saving_state(cv::Mat * image_array)
{
    printf("starting video write...\n");
    string gst_save = "appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw,format=RGBA ! nvvidconv ! nvv4l2h264enc ! h264parse ! qtmux ! filesink location=vid.h264";

    char filename[30];
    sprintf(filename, "img.jpg");

    cv::imwrite(filename,image_array[20]);
    
    cv::VideoWriter video(gst_save,cv::CAP_GSTREAMER,(float) number_frames/record_seconds,cv::Size(display_width, display_height), true);
    
    for(int i = 0; i < number_frames; ++i)
    {
	video.write(image_array[i]);
    }
    video.release();

    detection_found = false;
    record_done = false;

    printf("############# RECORDING DONE ##############\n");

    return ok;
}
enum return_codes sending_state(void)
{
	// Set transceiver into transmit mode and send video and image.

	cout << "sending image..." << endl;

	/*
	FILE * fp = fopen("img.jpg", "r");
	send_file(nrf_spi_fd, fp, 0);
	fclose(fp);

	cout << "sending video..." << endl;

	fp = fopen("vid.h264", "r");
	send_file(nrf_spi_fd, fp, 1);
	fclose(fp);

	cout << "completed sending." << endl;

	*/
	return ok;
}
enum return_codes end_state(void)
{
    // Deallocating all resources.
    LogVerbose("detectnet: shutting down...\n");
    LogVerbose("camera stream: shutting down...\n");
    LogVerbose("I2C interface: shutting down...\n");

    //SAFE_DELETE(output);
    cap->release();
    free_resources(i2c_fd,img_buffer,net);
    close(nrf_spi_fd);

    LogVerbose("Object tracking shutdown complete.\n");

	return ok;
}
enum state_codes lookup_next_state(enum state_codes sc, enum return_codes rc)
{
	return lookup_table[sc][rc].next_state;
}
