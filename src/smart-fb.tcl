# create a new simulator object
set ns [new Simulator]

# define a finish proc
proc finish {} {
    global ns nf
    $ns flush-trace
    close $nf
    # leave this guy in, for longer simulations, this provides necessary feedback to let the user know the simulation is done
    puts "Simulation complete..."
    exit 0
}

# open an output file
set nf [open "smart-fb.tr" w]

# create some clients with a router and server
set client_a [$ns node]
set client_b [$ns node]
set router_a [$ns node]
set server_a [$ns node]

Queue/SMART/SMART-FB set lfactor_ 0.5
Queue/SMART/SMART-FB set threshold_ 0.8
Queue/SMART/SMART-FB set target_ 5

# initialize the traced variables
#   apparently for traced variables this is necessary, otherwise ns2 throws some warnings around for no reason
Queue/SMART/SMART-FB set curq_ 0
Queue/SMART/SMART-FB set d_exp_ 0.0

# create the links between the nodes
$ns duplex-link $client_a $router_a 2Mb 25ms DropTail
$ns duplex-link $client_b $router_a 1.5Mb 25ms DropTail
$ns duplex-link $router_a $server_a 1.7Mb 50ms SMART/SMART-FB 

# set a queue size of 32 for the LSTFCoDel link
$ns queue-limit $router_a $server_a 32

# trace the queue we are concerned about, do not use "ns trace-all" - this command traces activity on ALL nodes, we do NOT want that!!
$ns trace-queue $router_a $server_a $nf

# create some agents
set tcp [new Agent/TCP]
# set packet size to Ethernet
$tcp set packetSize_ 1500

# create a UDP agent
set udp [new Agent/UDP]
# set packet size to Ethernet
$udp set packetSize_ 1000

# create the tcp sink and udp null sink (although it can be used with tcp agents too!)
set sink [new Agent/TCPSink]
set null [new Agent/Null]

# attach the agents to the nodes
$ns attach-agent $client_a $tcp
$ns attach-agent $client_b $udp
$ns attach-agent $server_a $sink
$ns attach-agent $server_a $null

# connect the agents and the sinks
$ns connect $tcp $sink
$ns connect $udp $null

# create some traffic generators
set ftp [new Application/FTP]
set cbr [new Application/Traffic/CBR]

# link the traffic generators to the agents
$ftp attach-agent $tcp
$cbr attach-agent $udp

# schedule some traffic - Maybe having both traffic generators running for the length of the simulation is a bad idea??, maybe...
$ns at 0 "$ftp start"
# stagger the start time for the telnet agent
$ns at 300 "$cbr start"

# schedule finish at variable sim time
$ns at 300 "finish"

# run the simulation
$ns run
