# SMART
NS-2 and eventual C implementation of the SMART discipline of AI-based router forwarding techniques. These techniques should be simple enough to implement on modern routers with no issue.

## Introduction
Modern routers are now sufficiently powerful enough to utilize smarter queuing disciplines, however, many routers still employ so-called 'dumb' techniques like first-in first-out (FIFO), round robin (RR), deficit round robin (DRR), and priority queuing. Meanwhile there has been active research in the area of smarter queuing disciplines within the realm of active queue management (AQM) with notable contributions such as random early detection (RED) and controlled delay (CoDel). These algorithms are more portable than the one proposed here as they are built as wide-spectrum solutions targeting devices from wireless sensors up to commercial grade switching devices. The issue is a compromise. These algorithms are smart not because they apply artificial intelligence (AI), but because they use statistical methods and control theory to pseudo-intelligently control packet queues.

SMART is a proposed paradigm for controlling queuing delay by employing classical and modern techniques from AI such as reinforcement learning, Markov decision processes, and feature-based learning. This algorithm plans to employ advanced computational techniques to more intelligently make decisions regarding queuing delay.

## Abstract
SMART is meant to be implemented on routers not end hosts. There are better machine learning models for packet switching that exist for end hosts such as `Titan`. SMART implements a model whereby the router builds a policy over time for dealing with congestion. Congestion however is temporal in nature. In an ordinary computer network, packets occur in regular streams (as in TCP traffic) with intermittent bursts occuring at irregulated intervals (as in UDP traffic).
Therefore, SMART needs ways to deal with the varying nature of service in the Internet.

SMMART implements a utility-based learning model where the agent tries to consistently maximize its reward from packet service. The model rewards the agent using a relationship between packet delay and persistent congestion. Packet delay is easy to measure, however persistent congestion is more difficult to deal with. Persistent congestion is modeled using an additive increase, multiplicative decrease model whereby persistently high delay exhibits a linear
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

+ `estimate(e, s, a) -> e`: Produces an estimate value, e, from the previous estimate, e and a sample value s, using the learning parameter, a. The learning parameter must be in the range [0, 1].
+ `normalize(v, min, max) -> w`: Produces a normalized value, w, in the range [0, 1] given a value 
+ `reward(s, s', a) -> r`: Determines the reward, r, for transitioning from state, s, to state, s', using the action, a.
+ `successors(s, A) -> S`: Generates a set of successor states, S, that from a state, s, using a set of actions, A.
+ `transition(s, a) -> s'`: Applies the action, a, to state, s, to produce a new state, s'.

## Models
SMART defines and implements three queuing models based on the above theory. These models are inspired from classical and modern AI machine learning techniques and are as follows:

1. SMART-RL: SMART queuing discipline that implements techniques from reinforcement learning. 
2. SMART-MDP: SMART queuing disicpline that implements techniques from Markov Decision Processes.
3. SMART-FB: SMART queuing discipline that implements techniques from Feature-based learning.

These models were carefully selected to ensure that the router only needs to maintian information on the previous state for decision making.

## Implementation Status
SMART is still in the early design stages. I would ask that if you want to contribute to this project please abide by the GPL-3.0 license. I would also appreciate that if you make any meaningful contributions to the project that you contribute them back to this code base. Please refrain from implemeting this software in the discrete network simulator NS-2, I am already handling it. That said, the implementation provided here requires the NS-2 discrete network
simulator in order to run the SMART implementation.
