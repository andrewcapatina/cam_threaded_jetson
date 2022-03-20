/**
 *  Andrew Capatina 
 *  Ad-hoc Defense
 * 
 *  Description:
 *      Main file that spawns threads.
 * 
 * */

#include <iostream>
#include <pthread.h>

#include "obj-tracker/obj-tracker.h"

using namespace std;

int main()
{
    pthread_t thread_obj_track;
    void * status;

    int rc = 0;

    pthread_attr_t attr;
    size_t stack_size;


    rc = pthread_create(&thread_obj_track, NULL, object_tracker,(void *)0);

    cout << rc << endl;

	
    pthread_join(thread_obj_track, &status);

    return 0;
}
