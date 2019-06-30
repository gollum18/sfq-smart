# name: smart-rl.tcl
# since: 06/30/2019
# author: Christen Ford <c.t.ford@vikes.csuohio.edu>
# purpose: This simulation allows for testing the SMART-RL AQM algorithm

# define a usage procedure
proc usage {err} {
    puts "Usage: ns smart-rl.tcl \[alpha\] \[discount\] \[rounds\] \[runtime\]"
    puts [format "Error: %s" $err]
    exit -1
}

# ensure that the proper number of arguments were entered
if {$argc != 4} {
    [usage "Invalid number of commands passed!"]
}

# get the parameters for the simulation
set alpha [lindex $argv 0]
set discount [lindex $argv 1]
set rounds [lindex $argv 2]
set runtime [lindex $argv 3]

# perform checks on the user supplied variables
if {$alpha < 0 || $alpha > 1} {
    [usage "alpha must be in the range of \[0, 1\]!"]
}

if {$discount < 0 || $discount > 1} {
    [usage "discount must be in the range of \[0, 1\]!"]
}

if {$rounds < 100} {
    [usage "the agent needs at least 100 training rounds!"]
}

# create a new simulator object
set ns [new Simulator]

# define different colros for data flows
$ns color 1 Blue
$ns color 2 Red

# create the output directory
file mkdir "out"

# open a trace file
set nf [open "out/smart-rl-trace-all.nam" w]
set nq [open "out/smart-rl-trace-q.nam" w]
$ns namtrace-all $nf

# define a finish procedure
proc finish {} {
    global ns nf nq
    $ns flush-trace
    close $nf
    close $nq
    exec nam "smart-rl-trace-all.nam" &
    exit 0
}

#n0->n2, n1->n2, n2->n3
# create the network nodes
for {set i 0} {$i < 4} {incr i} {
    set n($i) [$ns node]
}

# set the queue parameters
Queue/SMART/SMART-RL set alpha_ $alpha
Queue/SMART/SMART-RL set discount_ $discount
Queue/SMART/SMART-RL set rounds_ $rounds
Queue/SMART/SMART-RL set curq_ 0
Queue/SMART/SMART-RL set d_exp_ 0.0

# create links between the nodes
$ns duplex-link $n(0) $n(2) 2Mb 10ms DropTail
$ns duplex-link $n(1) $n(2) 2Mb 10ms DropTail
$ns duplex-link $n(2) $n(3) 1.7Mb 20ms SMART/SMART-RL

# set queue size for all links
$ns queue-limit $n(0) $n(2) 32
$ns queue-limit $n(1) $n(2) 32
$ns queue-limit $n(2) $n(3) 32

# trace the queue
$ns trace-queue $n(2) $n(3) $nq

# create a TCP agent
set tcp [new Agent/TCP]
# set packet size to Ethernet
$tcp set packetSize_ 1500

# create a udp agent
set udp [new Agent/UDP]
# set the packet size to Ethernet
$udp set packetSize_ 1500

# create the tcp sink and udp null sink
set sink [new Agent/TCPSink]
set null [new Agent/Null]

# attach the agents to the nodes
$ns attach-agent $n(0) $tcp
$ns attach-agent $n(1) $udp
$ns attach-agent $n(3) $sink
$ns attach-agent $n(3) $null

# connect the agents and the sinks
$ns connect $tcp $sink
$ns connect $udp $null

# create some traffic agents
set ftp [new Application/FTP]
set cbr [new Application/Traffic/CBR]

# link the traffic generators to the agents
$ftp attach-agent $tcp
$cbr attach-agent $udp

# schedule some traffic 
$ns at 0 "$ftp start"
$ns at 300 "$udp start"

#schedule the finish procedure
$ns at $runtime "finish"

# run the simulation
$ns run
