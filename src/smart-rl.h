/* SMART - Reinforcement learning implementation of SMART.
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

#ifndef NS_SMART_RL
#define NS_SMART_RL

#include "queue.h"
#include <cmath>
#include <cstdlib>
#include <map>
#include "agent.h"
#include "template.h"
#include "trace.h"

// Define variable state descriptors and bounds
#define NUM_STATES 5
#define Q_STEADY 0
#define Q_UNSTEADY 1
#define Q_FILLING 2
#define Q_CONGESTED 3
#define Q_IMMINENT 4

#define Q_STEADY_UB 0.20
#define Q_UNSTEADY_UB 0.40
#define Q_FILLING_UB 0.60
#define Q_CONGESTED_UB 0.80

// Defines actions to take when dequeing
#define ACTION_DEQUE 0
#define ACTION_DROP 1

// Define indexes for the queue state arrays
#define Q_LENGTH 0
#define Q_DELAY 1

// Necessary STL imports

using std::pair

// Define state encapsulation
typedef struct QueueState {
    TracedDouble curq_;
    TracedDouble d_exp_;
    double[2] avg_;
    double[2] min_;
    double[2] max_;
} queue_state;

class SmartRLQueue : public Queue {

    public:

        // Constructors
        
        SmartRLQueue();

    protected:
        
        // NS methods
        
        void enque(Packet*);                    // enqueues a packet
        Packet* deque();                        // dequeues a packet
        int command(int, const char*const*);    // creates the queue
        void reset();                           // resest the queue to like new
        void trace(TracedVar*);                 // traces a variable to tchan_

        // NS Variables
        
        PacketQueue* q_;            // the backing queue
        Tcl_Channel tchan_;         // the trace channel, where tracevars get written

        // Static state of the algorithm
        
        double alpha_;      // the averaging factor, in range [0, 1]
        double discount_;   // the discount factor, in range [0, 1]

        // Dynamic state of the algorithm
        
        QueueState state_;              // the state of the algorithm seen by new arrivals
        TracedDouble prev_curq_;        // the previous queue length (in bytes)
        TracedDouble prev_d_exp_;       // the previous experienced delay
        
        pair<int, double>> policy_[5];  // stores the agents policy for each state
        double trans_probs_[5];         // determines whether the agent follows policy, each entry should be in the range [0, 1] and indicates the agents chance to follow its optimal policy

    private:

        // Determines the approriate action to take when a packet is dequed
        int action(int);

        // Determines the adversarial (opposite) action of the deque action given
        int adversary(int action) = { return ACTION_DEQUE ? action == ACTION_DROP : ACTION_DEQUE; }

        // Determines the new average given the current average and a sample
        template <class T>
        double average(T, double, double);

        // Classifies the queue into one of 5 states that represent varying levels of congestion
        int classify();

        // Initializes the state of the algorithm
        void initialize();

        // Normalizes a data value given the value, min, and max.
        template <class T>
        double normalize(T, double, double);

        // Determines the reward when applying an action with a certain queue state.
        double reward(int, int);

        // Updates the state of the algorithm after dequeuing a packet
        void update(Packet*);

} smart_rl_queue;

#endif
