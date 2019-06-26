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
 
#include <sys/types.h>
#include "config.h"
#include "random.h"
#include "flags.h"
#include "delay.h"
#include "smart-fb.h"

static class SmartFBClass : public TclClass {
    public:
        SmartFBClass() : TclClass("Queue/SMART/SMART-FB") {}
        TclObject* create(int, const char*const*) {
            return (new SmartFBQueue);
        }
} class_smart_fb;

SmartFBQueue::SmartFBQueue() : tchan_(0) {
    bind("curq_", &curq_);          // current queue size in bytes
    bind("d_exp_", &d_exp_);        // current delay experienced in clock ticks
    bind("lfactor_", &lfactor_);    // the learning factor for the agent
    bind("target_", &target_);      // the target service delay
    bind("threshold_", &threshold_);// the threshold needed to reach dropping state 
    q_ = new PacketQueue();         // underlying queue
    pq_ = q_;
    reset();
}

void SmartFBQueue::reset() {
    curq_ = 0;
    d_exp_ = 0.;
    
    Queue::reset();
}

void SmartFBQueue::enque(Packet* pkt) {
    if (q_->length() >= qlim_) {
        drop(pkt);
    } else {
        HDR_CMN(pkt)->ts_ = Scheduler::instance().clock();
        q_->enque(pkt);
    }
}

Packet* SmartFBQueue::deque() {
    if (q_->length() == 0) {
        return 0;
    }
    // update the previous variables
    prev_curq_ = curq_;
    prev_d_exp_ = d_exp_;
    
    // grab a packet
    Packet* pkt = q_->deque();
    
    // update the current variables
    d_exp_ = Scheduler::instance().clock() - HDR_CMN(pkt)->ts_;
    curq_ = q_->byteLength();
    
    // determine the drift values
    double sdrift = drift((double)d_exp_, prev_d_exp_);
    double qdrift = drift((double)curq_, prev_curq_);
    
    // update metrics on min/max
    if (curq_ < getMin(FTR_CURQ)) {
        setMin(FTR_CURQ, curq_);
    } else if (curq_ > getMax(FTR_CURQ)) {
        setMax(FTR_CURQ, curq_);
    }
    
    if (d_exp_ < getMin(FTR_CURQ)) {
        setMin(FTR_DEXP, d_exp_);
    } else if (d_exp_ > getMax(FTR_CURQ)) {
        setMax(FTR_CURQ, d_exp_);
    }
    
    if (sdrift < getMin(FTR_SDRIFT)) {
        setMin(FTR_SDRIFT, sdrift);
    } else if (sdrift > getMax(FTR_SDRIFT)) {
        setMax(FTR_SDRIFT, sdrift);
    }
    
    if (qdrift < getMin(FTR_QDRIFT)) {
        setMin(FTR_QDRIFT, qdrift);
    } else if (qdrift > getMax(FTR_QDRIFT)) {
        setMax(FTR_QDRIFT, qdrift);
    }
    
    // update the feature set
    setFeature(FTR_DEXP, average(d_exp_, getFeature(FTR_DEXP), lfactor_));
    setFeature(FTR_CURQ, average(curq_, getFeature(FTR_CURQ), lfactor_));
    setFeature(FTR_SDRIFT, sdrift);
    setFeature(FTR_QDRIFT, qdrift);
    
    // update the weight set
    setWeight(FTR_DEXP, average(normalize((double)d_exp_, min_[FTR_DEXP], max_[FTR_DEXP]), getWeight(FTR_DEXP), lfactor_));
    setWeight(FTR_CURQ, average(normalize((double)curq_, min_[FTR_CURQ], max_[FTR_CURQ]), getWeight(FTR_CURQ), lfactor_));
    setWeight(FTR_SDRIFT, average(normalize(sdrift, min_[FTR_SDRIFT], max_[FTR_SDRIFT]), getWeight(FTR_SDRIFT), lfactor_));
    setWeight(FTR_QDRIFT, average(normalize(qdrift, min_[FTR_QDRIFT], max_[FTR_QDRIFT]), getWeight(FTR_QDRIFT), lfactor_));
    
    // dot product the feature and weight vectors
    double activator = 0;
    for (int i = 0; i < NUM_FEATURES; i++) {
        activator += getFeature(i) * getWeight(i);
    }
    
    // determine whether to enter dropping state
    if (sigmoid(activator) >= threshold_) {
        dropping_ = 1;
    }
    
    // drop packets to signal congestion
    while (dropping_) {
        pkt = q_->deque();
        double now = Scheduler::instance().clock();
        // drop until the observed packet delay falls below 
        //  the target service delay
        if (now - HDR_CMN(pkt)->ts_ 
                < target_) {
            dropping_ = 0;
        }
        // update views on queue
        prev_curq_ = curq_;
        prev_d_exp_ = d_exp_;
        curq_ = q_->byteLength();
        d_exp_ = now - HDR_CMN(pkt)->ts_;
        // drop the packet
        drop(pkt);
    }
    
    // assertion 
    if (q_->length() == 0) {
        return 0;
    }
    
    // grab an additional packet
    pkt = q_->deque();
    d_exp_ = Scheduler::instance().clock() - HDR_CMN(pkt)->ts_;
    
    // update views on queue
    prev_curq_ = curq_;
    prev_d_exp_ = d_exp_;
    curq_ = q_->byteLength();
    
    // return the packet
    return pkt;
}

int SmartFBQueue::command(int argc, const char*const* argv) {
    Tcl& tcl = Tcl::instance();
    if (argc == 2) {
        if (strcmp(argv[1], "reset") == 0) {
            reset();
            return (TCL_OK);
        }
    } else if (argc == 3) {
        // attach a file for variable tracing
        if (strcmp(argv[1], "attach") == 0) {
            int mode;
            const char* id = argv[2];
            tchan_ = Tcl_GetChannel(tcl.interp(), (char*)id, &mode);
            if (tchan_ == 0) {
                tcl.resultf("SmartFBQueue trace: can't attach %s for writing", id);
                return (TCL_ERROR);
            }
            return (TCL_OK);
        }
        // connect SmartFBQueue to the underlying queue
        if (!strcmp(argv[1], "packetqueue-attach")) {
            delete q_;
            if (!(q_ = (PacketQueue*) TclObject::lookup(argv[2])))
                return (TCL_ERROR);
            else {
                pq_ = q_;
                return (TCL_OK);
            }
        }
    }
    return (Queue::command(argc, argv));
}

void SmartFBQueue::trace(TracedVar* v) {
    const char *p;

    if (((p = strstr(v->name(), "curq")) == NULL) &&
        ((p = strstr(v->name(), "d_exp")) == NULL) ) {
        fprintf(stderr, "SMART-FB: unknown trace var %s\n", v->name());
        return;
    }
    if (tchan_) {
        char wrk[500];
        double t = Scheduler::instance().clock();
        if(*p == 'c') {
            sprintf(wrk, "c %g %d", t, int(*((TracedInt*) v)));
        } else if(*p == 'd') {
            sprintf(wrk, "d %g %g", t, double(*((TracedDouble*) v)));
        }
        int n = strlen(wrk);
        wrk[n] = '\n'; 
        wrk[n+1] = 0;
        (void)Tcl_Write(tchan_, wrk, n+1);
    }
}
