/*
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ias_drawer_executive/Pressure.h>


void Pressure::calcCenter(float pressure[], double &xcenter, double &ycenter)
{
    double xsum = 0;
    double ysum = 0;
    double psum = 0;
    for (int i = 7; i <= 21; ++i)
    {
        double x = ((i - 7) % 3) - 1;
        double y = (((i - 7) - ((i - 7) % 3)) / 3) - 2;
        //ROS_INFO("COORDS idx %i, x %f, y %f", i, x, y);
        xsum += x * pressure[i];
        ysum += y * pressure[i];
        psum += pressure[i];
    }
    if (psum < 5000)
        psum = 5000;

    if (psum != 0)
    {
        xsum /= psum;
        ysum /= psum;
    }
    xcenter = xsum;
    ycenter = ysum;
}


void Pressure::pressureCallback(const pr2_msgs::PressureState::ConstPtr& msg)
{
    if (zero_cnt_ <= 10)
    {
        zero_cnt_ ++;
        for (int i = 0; i < 22; i++)
        {
            r_sum[i] += msg->r_finger_tip[i];
            l_sum[i] += msg->l_finger_tip[i];
        }
        if (zero_cnt_ == 10)
        {
            pressure_mutex.lock();
            for (int i = 0; i < 22; i++)
            {
                r_zero[i] = r_sum[i] / zero_cnt_;
                l_zero[i] = l_sum[i] / zero_cnt_;
                //ROS_INFO("ZERO OF GRIPPER PRESSURE i %i r %f l %f", i, r_zero[i], l_zero[i]);
            }
            pressure_mutex.unlock();
        }
    }

    double r[2];
    double l[2];
    float r_act[22];
    float l_act[22];
    for (int i = 0; i < 22; i++)
    {
        r_act[i] = msg->r_finger_tip[i] - r_zero[i];
        l_act[i] = msg->l_finger_tip[i] - l_zero[i];
    }

    calcCenter(r_act, r[0], r[1]);
    calcCenter(l_act, l[0], l[1]);
    //ROS_INFO("CENTERS rx %f ry %f , lx %f ly %f", r[0], r[1], l[0], l[1]);

    //ROS_INFO("DIFFS x %f y %f" , r[0] - l[0], r[1] - l[1]);

    pressure_mutex.lock();
    r_center[0] = r[0];
    r_center[1] = r[1];
    l_center[0] = l[0];
    l_center[1] = l[1];
    for (int i = 0; i < 22; i++)
    {
        r_curr[i] = msg->r_finger_tip[i];
        l_curr[i] = msg->l_finger_tip[i];
    }
    pressure_mutex.unlock();

    for (int finger_tip = 0; finger_tip < 2; ++finger_tip)
    {
        for (int nrSensor = 0; nrSensor < 22; ++nrSensor)
        {
            //if ((touched[finger_tip][nrSensor]==1) && (val[finger_tip][nrSensor]>=val_prev[finger_tip][nrSensor]))
                //slope[finger_tip][nrSensor]=1;
            int val = (finger_tip ? l_curr[nrSensor] : r_curr[nrSensor]);

            slope[finger_tip][nrSensor]=(double)(val-val_prev_prev[finger_tip][nrSensor])/val_prev_prev[finger_tip][nrSensor];

            val_prev_prev[finger_tip][nrSensor]=val_prev[finger_tip][nrSensor];
            val_prev[finger_tip][nrSensor]=val;

            if (slope[finger_tip][nrSensor]>0.05) {
                touched[finger_tip][nrSensor]++;
            }
        }
    }
    initialized = true;
}

Pressure::Pressure(int side)
{
    sub_ = n_.subscribe((side == 0) ? "/pressure/r_gripper_motor" : "/pressure/l_gripper_motor", 1000, &Pressure::pressureCallback,this); // zzz
    zero_cnt_ = 0;
    side_ = side;
    for (int i = 0; i < 22; i++)
    {
        r_sum[i] = 0;
        l_sum[i] = 0;
        r_zero[i] = 0;
        l_zero[i] = 0;
        val_prev[0][i] = 0;
        val_prev[1][i] = 0;
        val_prev_prev[0][i] = 0;
        val_prev_prev[1][i] = 0;
        touched[0][i] = 0;
        touched[1][i] = 0;
    }
    initialized = false;
}

Pressure::~Pressure()
{
    //delete sub_;
}

Pressure *Pressure::instance_[] = {0,0};

Pressure *Pressure::getInstance(int side)
{
    if (!instance_[side])
    {
        instance_[side] = new Pressure(side);
    }
    return instance_[side];
}

void Pressure::reset()
{
    pressure_mutex.lock();
    zero_cnt_ = 0;
    pressure_mutex.unlock();
    ros::Rate rate(25.0);

    long act_cnt = 0;
    while (act_cnt < 10)
    {
        rate.sleep();
        ros::spinOnce();
        pressure_mutex.lock();
        if (act_cnt < zero_cnt_)
            ROS_INFO("Waiting for zeroing of pressure sensors");
        act_cnt = zero_cnt_;
        pressure_mutex.unlock();
    }

}

void Pressure::getCenter(float *r, float *l)
{
    pressure_mutex.lock();
    r[0] = r_center[0];
    r[1] = r_center[1];
    l[0] = l_center[0];
    l[1] = l_center[1];
    pressure_mutex.unlock();
}

void Pressure::getCurrent(float r[], float l[], bool zero)
{
    ros::spinOnce();
    pressure_mutex.lock();
    for (int i = 0; i < 22; i++)
    {
        if (zero)
        {
            r[i] = r_curr[i] - r_zero[i];
            l[i] = l_curr[i] - l_zero[i];
        }
        else
        {
            r[i] = r_curr[i];
            l[i] = l_curr[i];
        }
    }
    pressure_mutex.unlock();
}


void Pressure::getInside(float &r, float &l, bool zero)
{
    r = 0;
    l = 0;
    ros::Rate rate(5);
    while (!initialized)
    {
        rate.sleep();
        ros::spinOnce();
    }
    for (int i = 7; i < 22; ++i)
    {
        if (zero)
        {
            r+= r_curr[i] - r_zero[i];
            l+= l_curr[i] - l_zero[i];
        }
        else
        {
            r+= r_curr[i];
            l+= l_curr[i];
        }
    }
}




void Pressure::getFront(float &r, float &l, bool zero)
{
    r = 0;
    l = 0;
    ros::Rate rate(5);
    while (!initialized)
    {
        rate.sleep();
        ros::spinOnce();
    }
    for (int i = 3; i <= 4 ; ++i)
    {
        if (zero)
        {
            r+= r_curr[i] - r_zero[i];
            l+= l_curr[i] - l_zero[i];
        }
        else
        {
            r+= r_curr[i];
            l+= l_curr[i];
        }
    }
}



void Pressure::getInsideTouched(float &r, float &l)
{
    r = 0;
    l = 0;
    ros::Rate rate(5);
    while (!initialized)
    {
        rate.sleep();
        ros::spinOnce();
    }
    for (int i = 7; i < 22; ++i)
    {
            r+= touched[0][i];
            l+= touched[1][i];
    }
}

void Pressure::getFrontTouched(float &r, float &l)
{
    r = 0;
    l = 0;
    ros::Rate rate(5);
    while (!initialized)
    {
        rate.sleep();
        ros::spinOnce();
    }
    for (int i = 3; i < 4; ++i)
    {
            r+= touched[0][i];
            l+= touched[1][i];
    }
}
