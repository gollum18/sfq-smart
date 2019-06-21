# SMART
NS-2 and eventual C implementation of the SMART discipline of AI-based router forwarding techniques. These techniques should be simple enough to implement on modern routers with no issue.

## Introduction


## Abstract
SFQ-Smart is meant to be implemented on routers not end hosts. There are better machine learning models for packet switching that exist for end hosts such as `Titan`. SFQ-Smart implements a model whereby the router builds a policy over time for dealing with congestion. Congestion however is temporal in nature. In an ordinary computer network, packets occur in regular streams (as in TCP traffic) with intermittent bursts occuring at irregulated intervals (as in UDP traffic).
Therefore, SFQ-Smart needs ways to deal with the varying nature of service in the Internet.

SFQ-Smart implements a utility-based learning model where the agent tries to consistently maximize its reward from packet service. The model rewards the agent using a relationship between packet delay and persistent congestion. Packet delay is easy to measure, however persistent congestion is more difficult to deal with. Persistent congestion is modeled using an additive increase, multiplicative decrease model whereby persistently high delay exhibits a linear
relationship on the model while low or no congestion exhibits an inverse exponential relationship on the model. 

## Theory
Let *Z* be the state of a statistical multiplexer with the following characteristics:

+ *r*: The packet arrival rate (always variable).
+ *s*: The packet service rate (almost always fixed).
+ *q*: The packet queue.

Then the system load is given by the following equation: *r*/*s*=*p* where *p* is the Greek symbol, rho.

According to queuing theory, as *p* approaches 1, *q* will start to experience more and more delay. If at any time, *p* exceeds 1, then the *q* will grow indefinitely and packets will be dropped.

Also, for simplicities sake, assume that the multiplexer has a single input port and a single output port.

Additionally, let the following characteristics be observable from *Z*:

+ *d*: The most recent packet delay.
+ *n*: The number of packets in the queue.
+ *l*: The most recent packet size.

Finally, assume that the router also tracks the average, min, and max of the above variables.

With that out of the way, we can define the algorithm.

Let *A* represent the SFQ-Smart algorithm which defines the following functions:

+ `estimate(e, s, a, b) -> e`: Produces an estimate value, e, from the previous estimate, e, using the weighting variables, a and b.
+ `evalulate(s, a) -> e`: Produces an evaluation (a score), e, of a state s using action a.
+ `normalize(v, min, max) -> w`: Produces a normalized value, w, in the range [0, 1] given a value 
+ `reward(s, s', a) -> r`: Determines the reward, r, for transitioning from state, s, to state, s', using the action, a.
+ `successors(s, A) -> S`: Generates a set of successor states, S, that from a state, s, using a set of actions, A.
+ `transition(s, a) -> s'`: Applies the action, a, to state, s, to produce a new state, s'.

## Models
SMART defines and implements three queuing models based on the above theory. These models are inspired from classical and modern AI machine learning techniques and are as follows:

1. SMART-RL: SMART queuing discipline that implements techniques from reinforcement learning. 
2. SMART-MDP: Smart queuing disicpline that implements techniques from Markov Decision Processes.
3. SMART-FB: SMART queuing discipline that implements techniques from Feature-based learning.

These models were carefully selected to ensure that the router only needs to maintian information on the previous state for decision making.

## Status
SMART is still in the early design stages. I would ask that if you want to contribute to this project please abide by the GPL-3.0 license. I would also appreciate that if you make any meaningful contributions to the project that you contribute them back to this code base.
