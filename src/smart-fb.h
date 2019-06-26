/* SMART - Feature-based learning implementation of SMART.
 * Copyright (C) 2019 Christen Ford <c.t.ford@vikes.csuohio.edu>
 *
 * This software uses portions of the CoDel implementation for NS-2:
 * Copyright (C) 2011-2012 Kathleen Nichols <nichols@pollere.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */ 

/* This program is intended for use with the Network Simulator 2 
 * discrete event simluator. I make no claims that this software will 
 * function with future versions of NS such as NS-3 or NS-4. NS-2 was 
 * chosen as it is still the primary and most well-respected 
 * simulator for networking research.
 *
 * To utilize SMART in your research, you must acquire a source 
 * distribution for NS-2 and compile it with all SMART implementation 
 * files (smart-*.cc) pointed to by NS-2s makefile. I cannot assist 
 * you in compiling NS-2 as it differs per platform. 
 *
 * Note that NS-2 is known not to work correctly on Linux deployments 
 * such as CygWin.
 */

#ifndef NS_SMART_FB
#define NS_SMART_FB

#include "queue.h"
#include <cmath>
#include <cstdlib>
#include "agent.h"
#include "template.h"
#include "trace.h"

#define NUM_FEATURES 4

// features indexes
#define FTR_DEXP 0
#define FTR_CURQ 1
#define FTR_SDRIFT 2
#define FTR_QDRIFT 3 

class SmartFBQueue : public Queue {

    public:
        
        // constructors
        SmartFBQueue();
        
        // accessor methods
        double getFeature(int feature) {
            return features_[feature];
        }
        double getWeight(int feature) {
            return weights_[feature];
        }
        double getMin(int feature) {
            return min_[feature];
        }
        double getMax(int feature) {
            return max_[feature];
        }
        
        // setter methods
        void setFeature(int feature, double value) { 
            features_[feature] = value;
        }
        void setWeight(int feature, double value) {
            weights_[feature] = value;
        }
        void setMin(int feature, double value) {
            min_[feature] = value;
        }
        void setMax(int feature, double value) {
            max_[feature] = value;
        }
        
    protected:
    
        // Queue method overrides
        
        void enque(Packet*);    // enqueues a packet
        Packet* deque();        // dequeues a packet

        // NS-specific methods

        int command(int, const char*const*);    // used to setup the queue
        void reset();                           // resets the queue to intial state
        void trace(TracedVar*);                 // traces a traced variable to the trace channel
        
        // NS variables
        
        PacketQueue* q_;        // underlying FOF queue
        Tcl_Channel tchan_;     // place to write trace records
        TracedDouble curq_;     // current qlen seen by arrivals
        TracedDouble d_exp_;    // delay seen by most recently dequeued packet
    
        // dynamic state of the algorithm
        double prev_curq_;     // the previous queue length
        double prev_d_exp_; // the previous service delay
        
        double features_[NUM_FEATURES]; // the feature vector
        double weights_[NUM_FEATURES];  // the weighting vector
        double min_[NUM_FEATURES];      // the min values tracked
        double max_[NUM_FEATURES];      // the amx values tracked
        
        double lfactor_;    // the learning rate
        double target_;     // the target service delay
        double threshold_;  // the threshold needed to enter dropping state
        
        int dropping_;      // whether we are in dropping state
        
    private:
        
        /**
         * Determines the weighted average of an observed variable 
         * over time.
         * @param sample The most recent sample of the observed 
         * variable.
         * @param avg The current weighted average.
         * @param lfactor The learning factor used to discount the 
         * new average.
         * @return 
         */
        template <class T>
        double average(T sample, double avg, double lfactor) {
            return (1-lfactor)*avg+lfactor*sample;
        }
        
        /**
         * Determines the drift between a new and prior sample of 
         * an observed variable.
         * @param x The new sample.
         * @param y The prior sample.
         * @returns y - x
         */
        template <class T>
        T drift(T x, T y) {
            return y - x;
        }
        
        /**
         * Defines the L1 loss function used to determine the 
         * accuracy of a ML algorithm.
         */
        template <class T>
        T L1(T prediction, T actual) {
            return abs(actual - prediction);
        }
        
        /**
         * Defines the L2 loss function used to determine the 
         * accuracy of a ML  algorithm. 
         */
        template <class T>
        T L2(T prediction, T actual) {
            return (actual - prediction)*(actual - prediction);
        }
        
        /**
         * Normalizes a data value to be in the range [0, 1].
         * @param x The data value to normalize.
         * @param min The minimum observed data value seen.
         * @param max The maximum observed data value seen.
         * @returns A value in the range [0, 1].
         */
        template <class T>
        double normalize(T x, T min, T max) {
            // the cast is necessary to prevent truncation from 
            // T being an integer
            return (x-min)/((double)(max-min));
        }
        
        /**
         * Computes the sigmoid function over a given value. Sigmoid 
         * functions are commonly used as activator functions for 
         * neurons in neural networks.
         * @param x The value to compute the sigmoid function over.
         */
        template <class T>
        double sigmoid(T x) {
            return 1.0/(1+exp(-1*x));
        }
        
};

#endif
