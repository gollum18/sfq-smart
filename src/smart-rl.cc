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

#include <sys/types.h>
#include "config.h"
#include "random.h"
#include "flags.h"
#include "delay.h"
#include "smart-rl.h"

static class SmartRLClass : public TclClass {
    public:
        SmartRLClass() : TclClass("Queue/SMART/SMART-RL") {}
        TclObject* create(int, const char*const*) {
            return (new SmartRLQueue);
        }
} class_smart_rl;

// Constructor implementation

/**
 *
 */
SmartRLQueue::SmartRLQueue() : tchan_(0) {
    
}

// NS method implementation

/**
 *
 */
void SmartRLQueue::reset() {

}

/**
 * Enques a packet if there is room in the queue, otherwise the packet will drop.
 * @pkt A reference to a Packet object.
 */
void SmartRLQueue::enque(Packet* pkt) {
    if (q_->length() >= qlim_) {
        drop(pkt);
    } else {
        q_->enque(pkt);
    }
}

/**
 * Deques a packet from the queue and runs a single iteration of the algorithm.
 */
Packet* SmartRLQueue::deque() {
    // Note that I always put these in to guard against very stupid behavior encoded into NS-2s Queue class deque routine. 
    // In said routine, if the queue is empty, NS-2 will error out when deque is called and program execution will terminate.
    if (q_->length == 0) {
        return 0;
    }

    // Deque a packet
    Packet* pkt =  q_->deque();

    // Update the state of the algorithm
    update(pkt);

    // determine the state of the algorithm
    int state = classify();

    // Determine the action to take
    int action = action(state);

    // Check to see if we need to deque another packet
    if (action == ACTION_DROP) {
        // technically, routers only need to drop one packet to signal congestion
        //  However: the smarter idea may be to drop multiple packets to signal
        //  to multiple senders to back off
        // note that the algorithm could be modified to ECN packets instead
        drop(pkt);
        // Check to see if we can deque another packet
        if (q_->length > 0) {
            // deque and update the algorithm again to account for it
            pkt = q_->deque();
            update(pkt);
            state = classify();
        } else { return 0; }
    }

    // Determine the reward from the state and action
    double sa_reward = reward(state, action);

    // update the policy if necessary
    if (reward > policy_[state].second) {
        policy_[state].first = action;
        policy_[state].second = sa_reward;
    }

    // TODO: We still need to update the transition probability

    // discount the reward for each state, this is to encourage the agent
    //  to find potentially better options
    for (int i = 0; i < NUM_STATES; i++) {
        policy_[i].second = average(policy_[i].second, reward, discount_);
    }

    // Return the packet
    return pkt;
}

/**
 *
 */
int SmartRLQueue::command(int argc, const char*const* argv) {

}

/**
 *
 */
void SmartRLQueue::trace(TracedVar* v) {

}

// Utility method implementation

/**
 * Determines the appropriate action to take when dequeing a packet.
 */
int SmartRLQueue::action(int state) {
    if (Random::uniform(0, 1) < trans_probs_[state]) {
        // Follow policy
        return policy_[state].first;
    } else {
        // Follow opposing policy
        return adversary(policy_[state].first);
    }
}

/**
 * Determines the weighted average of a variable given its current value, a new value
 * and an averaging rate.
 * @param x The current value.
 * @param y The newly observed sample value.
 * @param rate The averaging rate.
 * @return A weighted average of a variable.
 */
double SmartRLQueue::average(double x, double y, double rate) {
    return (1 - rate) * x + rate * y;
}

/**
 * Determines the service class that the queue falls into.
 */
int SmartRLQueue::classify() {
    // determine normalized values for the length and delay 
    double len_norm = normalize(state_.curq_, state_.min_[Q_LENGTH], state_.max_[Q_LENGTH]);
    double del_norm = normalize(state_.d_exp_, state_.min_[Q_DELAY], state_.max_[Q_DELAY]);
    // take their average, they should still be in the range [0, 1]
    double avg = (len_norm + del_norm) / 2.0;
    if (avg < Q_STEADY_UB) {
        return Q_STEADY;
    } else if (avg < Q_UNSTEADY_UB) {
        return Q_UNSTEADY;
    } else if (avg < Q_FILLING_UB) {
        return Q_FILLING;
    } else if (avg < Q_CONGESTED_UB) {
        return Q_CONGESTED;
    } else {
        return Q_IMMINENT;
    }
}

/**
 * Initializes the algorithm to initial state.
 */
void SmartRLQueue::initialize() {
    
}

/**
 * Normalizes a value so it is in the range of [0, 1].
 * @param x The value to normalize.
 * @param min The minimum observed value.
 * @param max The maximum observed value.
 * @return A normalized value in the range [0, 1].
 */
double SmartRLQueue::normalize(double x, double min, double max) {
    if (max - min == 0) {
        // This is a dangerous compromise
        // TODO: Determine the optimal value to return here as it will adversely affect the algorithm if it is not carefully chosen
        return 0.5;
    }
    return (x - min) / (max - min);
}

/**
 * Determines a reward for the agent taking a specific action from a given state.
 * @param action The action the agent took during the current iteration.
 */
double SmartRLQueue::reward(int state, int action) {
    double reward = 0.0;
    // determine whether dropping was a good idea
    if (action == ACTION_DROP) {
        reward += 100 * state;
    } else {
        // the hardcoded 4 here is because there are 5 states indexed 0 -> 4
        reward -= 100 * (4 - state);
    }
    // factor in the queue length
    if (action == ACTION_DROP) {
        
    } else {

    }
    // factor in the delay
    if (action == ACTION_DROP) {

    } else {

    }
    return reward;
}

/**
 * Updates the state of the algorithm after a packet is dequeued. This is pretty 
 * much a transition function.
 * @param pkt The packet dequeued from the queue.
 */
void SmartRLQueue::update(Packet* pkt) {
    // update the tracked variable state
    prev_curq_ = state_.curq_;
    prev_d_exp_ = state_.d_exp_;
    state_.curq_ = q_->byteLength();
    state_.d_exp_ = Scheduler::instance().clock() - HDR_CMN(pkt)->ts_;

    // update the average values
    state_.avg_[Q_LENGTH] = average(state_.avg_[Q_LENGTH], state_.curq_);
    state_.avg_[Q_DELAY] = average(state_.avg_[Q_DELAY], state_.d_exp_);
    
    // update the min and max if needed
    if (state_.curq_ < state_.min_[Q_LENGTH]) {
        state_.min_[Q_LENGTH] = state_.curq_;
    } else if (state_.curq_ > state_.max_[Q_LENGTH]) {
        state_.max_[Q_LENGTH] = state_.curq_;
    }

    if (state_.d_exp_ < state_.min_[Q_DELAY]) {
        state_.min_[Q_DELAY] = state_.d_exp_;
    } else if (state_.d_exp_ > state_.max_[Q_DELAY]) {
        state_.max_[Q_DELAY] = state_.d_exp_;
    }
}
