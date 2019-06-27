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
#include <utility>
#include "agent.h"
#include "template.h"
#include "trace.h"

// Define state encapsulation
typedef struct QueueState {
    TracedInt curq_;
    TracedDouble d_exp_;
    double[2] avg_;
    double[2] min_;
    double[2] max_;
} queue_state;

// Define variable state descriptors and bounds
#define Q_STEADY 0
#define Q_UNSTEADY 1
#define Q_FILLING 2
#define Q_CONGESTED 3
#define Q_IMMINENT 4

#define Q_STEADY_UB 0.20
#define Q_UNSTEADY_UB 0.40
#define Q_FILLING_UB 0.60
#define Q_CONGESTED_UB 0.80

// Define the actions to take when receiving a packet
#define ACTION_ENQUE 0
#define ACTION_DROP 1

// Necessary STL imports

using std::map
using std::pair

class SmartRLQueue : public Queue {

    public:

        // Constructors
        
        SmartFBQueue();

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
        TracedInt prev_curq_;       // the previous queue length (in bytes)
        TracedDouble prev_d_exp_;   // the previous delay experienced

        // Static state of the algorithm
        
        double discount_;   // the discount factor

        // Dynamic state of the algorithm
        QueueState state_;          // the state of the algorithm seen by new arrivals
        TracedInt prev_curq_;       // the previous queue length (in bytes)
        TracedDouble prev_d_exp_;   // the previous experienced delay
        
        /* Notes on this structure:
         *  1.) The key value is a pair consisting of two Q_* states: the first for 
         *  the service delay, the second for the queue length.
         *  2.) The value is a pair consisting of: an action, and an associated 
         *  reward, in that order.
         * The agents goal is to maximize its total reward
         */
        map<pair<int, int>, pair<int, double>> states_;     // the state/action pairs

    private:

        // Determines the approriate action to take when receiving a packet
        int action(Packet*);

        // Classifies service delay seen by arrivals
        int classify_delay();

        // Classifies queue length seen by arrivals
        int classify_length();

        // Classifies the queue itself
        int classify_queue(int, int);

        // Initializes the state of the algorithm
        void initialize();

        // Determines the reward when applying an action during enquing
        double reward(int);

        // Updates the state of the algorithm after dequeuing a packet
        void update(Packet*);

} smart_rl_queue;

#endif
