# sfq-smart
NS-2 and eventual C Implementation of modified statistical fair queuing with techniques inspired from machine learning and modern AQM methodology.


## What is sfq-smart?
SFQ-Smart is an idea I came up with while performing in-depth research on machine learning techniques. I thought why not apply these techniques to 
create a smart AQM? Certainly, modern routers and switches have sufficiently powerful hardware to run at least a simple AI-inspired queuing mechanism.

The goal is to create an AQM that learns to differentiate good queue from bad queue and can proactively respond to congestion before it occurs. The hope 
is that after deployement the agent will act independtly from all other devices in the network. It should only be concerned with whats happening locally.

SFQ-Smart is still in the design stage. An initial prototype will be constructed and refined in NS-2 to gauge whether an implementation is worth pursuing 
within the Linux kernel.
