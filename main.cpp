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
