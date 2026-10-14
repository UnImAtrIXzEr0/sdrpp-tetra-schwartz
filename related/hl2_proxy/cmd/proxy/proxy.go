package main

import (
	"bufio"
	"errors"
	"flag"
	"fmt"
	"github.com/google/gopacket"
	"github.com/google/gopacket/pcapgo"
	"io"
	"log"
	"math"
	"net"
	"net/netip"
	"os"
	"os/exec"
	"runtime"
	"runtime/pprof"
	"strings"
	"sync"
	"syscall"
	"time"
)

var backendAddr netip.AddrPort
var clientCount = 0
var serverCount = 0
var frontend *net.UDPConn
var backendConn *net.UDPConn
var frontendAddr *net.UDPAddr
var lastClientReport = int64(0)
var lastServerReport = int64(0)
var APPTXsuspended = false

type TranscieverState struct {
	registers              [256][4]byte
	knownregs              [256]bool
	revpower               [10]uint16
	fwdpower               [10]uint16
	sendPackets            [][]byte
	snapshotPacketBuffer   []byte
	lowStateBits           byte
	fillLevel              int
	fillLevelTime          time.Time
	sentSinceFillLevelTime int // towards TRX
	swr                    float64
	swrFinal               byte
}

type ClientState struct {
	addr                    netip.AddrPort
	udpSendSequence         uint32
	udpInsertions           uint32 // added to sequence
	transmit                bool
	transmitChangeTime      time.Time
	registers               [256][4]byte
	knownregs               [256]bool
	lastReceived            time.Time
	lastTrueReceived        time.Time
	last83                  byte
	lastSentTowards         time.Time
	lastSentRegister        int
	transmitFramesSentToTRX int // how many frames sent to trx since transmitChangeTime if transmitting
}

var transcieverState TranscieverState
var clientState ClientState

type LogMessage struct {
	when  time.Time
	what  string
	fatal bool
}

type MyLog struct {
	pipe chan LogMessage
}

var logg MyLog

func (self *MyLog) Println(x ...any) {
	self.pipe <- LogMessage{time.Now(), fmt.Sprintln(x...), false}
}

func (self *MyLog) Printf(f string, x ...any) {
	self.pipe <- LogMessage{time.Now(), fmt.Sprintf(f, x...), false}
}

func (self *MyLog) PrintfStd(f string, x ...any) {
	self.pipe <- LogMessage{time.Time{}, fmt.Sprintf(f, x...), false}
}

func (self *MyLog) Fatal(x ...any) {
	self.pipe <- LogMessage{time.Now(), fmt.Sprintln(x...), true}
	time.Sleep(100000 * time.Second)
}

func (s *TranscieverState) updateStateFromFrame(ctm time.Time, frame []byte) {
	// wants 512 bytes in frame
	if frame[0] == 0x7F && frame[1] == 0x7F && frame[2] == 0x7F {
		// sync ok.
		var control [5]byte
		copy(control[:], frame[3:8])
		//receivers := 1
		// 8 bytes above (3=sync, 5=control.
		//niq_samples := (512 - 8) / ((receivers * 6) + 2) // how samples are packed, so it's one sample size
		s.updateStateFromControl(ctm, control)
	}
}

func (s *TranscieverState) updateStateFromUDP(ctm time.Time, udpData []byte) {
	if udpData[0] == 0xEF && udpData[1] == 0xFE {
		if udpData[2] == 2 || udpData[2] == 3 { // discovery response
			logg.Println("TRX -> APP sent 28 code as information about proxy")
			dup := make([]byte, len(udpData))
			copy(dup, udpData)
			dup[2] = 28 // indicates extended proxy (hl_2 proxy) presence.
			s.sendPackets = append(s.sendPackets, dup)
		}
		if udpData[2] == 1 { // iq data stream
			endpoint := udpData[3]
			if endpoint == 6 {
				s.updateStateFromFrame(ctm, udpData[8:8+504])
				s.updateStateFromFrame(ctm, udpData[520:520+504])
			}
		}
	}
}

func (s *TranscieverState) getFillLevel(ctm time.Time) int {
	if s.fillLevel < 1 {
		s.fillLevel = 0
	}
	timeSinceMicros := int(ctm.Sub(s.fillLevelTime).Microseconds()) // microseconds
	samplesPerPacket := 2 * 63
	sentRate := 48000
	packetDuration := (1000000 * samplesPerPacket) / sentRate // microseconds
	packetsTransmittedToAir := timeSinceMicros / packetDuration
	packetsAddedSinceReport := s.sentSinceFillLevelTime
	return s.fillLevel - packetsTransmittedToAir + packetsAddedSinceReport
}

func findMaxUint(arr []uint16) uint16 {
	m := arr[0]
	for i := 1; i < len(arr); i++ {
		if arr[i] > m {
			m = arr[i]
		}
	}
	return m
}

func (s *TranscieverState) updateStateFromControl(ctm time.Time, control [5]byte) {
	reg := (control[0] >> 3) & 0x1F
	s.lowStateBits = control[0] & 0x7
	s.knownregs[reg] = true
	s.registers[reg][0] = control[1]
	s.registers[reg][1] = control[2]
	s.registers[reg][2] = control[3]
	s.registers[reg][3] = control[4]

	switch reg {
	case 0:
		recovery := (control[3] & 0xC0) >> 6
		if recovery == 3 {
			s.fillLevel = 10000 // overflow
		} else if recovery == 2 {
			s.fillLevel = -1 // underflow
		} else {
			msb := control[3] & 0b00111111
			s.fillLevel = (int(msb) * 16) / 48
		}
		s.sentSinceFillLevelTime = 0
		s.fillLevelTime = ctm
	case 1:
		//adc := ((uint16(control[1])) << 8) | (uint16(control[2]) & 0xFF)
		//thisTemp := (3.26*(float64(adc)/4096.0) - 0.5) / 0.01
		//// Exponential moving average filter
		//alpha := 0.7
		//s.temperature = (alpha * thisTemp) + (1-alpha)*s.temperature
		copy(s.fwdpower[0:9], s.fwdpower[1:10])
		s.fwdpower[9] = (uint16(control[3]) << 8) | uint16(control[4]) // from Alex or Apollo
	case 2:
		copy(s.revpower[0:9], s.revpower[1:10])
		s.revpower[9] = (uint16(control[1]) << 8) | uint16(control[2]) // from Alex or Apollo
		//AIN3 := (control_in[3] << 8) + control_in[4];                                 // from Pennelope or Hermes
	}

	{
		constant1 := 3.3
		constant2 := 1.4
		fwd_cal_offset := uint16(6)

		fwd_power := findMaxUint(s.fwdpower[:]) - fwd_cal_offset
		v1 := float64(fwd_power) / 4095.0 * constant1
		fwd := (v1 * v1) / constant2

		rev := 0.0
		if fwd_power != 0 {
			v1 = float64(findMaxUint(s.revpower[:])) / 4095.0 * constant1
			rev = (v1 * v1) / constant2
		}

		thisSwr := (1 + math.Sqrt(rev/fwd)) / (1 - math.Sqrt(rev/fwd))
		if thisSwr < 0.0 {
			thisSwr = 1.0
		}

		if fwd < 0.05 {
			s.swr = 1
		} else {
			if math.IsNaN(s.swr) || math.IsInf(s.swr, 1) || math.IsInf(s.swr, -1) {
				s.swr = 1 // fix previous value
			}
		}
		alpha := 0.7
		s.swr = (alpha * thisSwr) + (1-alpha)*s.swr
		if s.swr < 0 || s.swr > 10 {
			s.swr = 0
		}

		s.swrFinal = byte(s.swr * 10)

		s.knownregs[28] = true
		s.registers[28][0] = s.swrFinal
		s.registers[28][1] = byte(fwd * 10)
		s.registers[28][2] = byte(rev * 10)
		s.registers[28][3] = 0

		//if s.swr != 10 {
		//	logg.PrintfStd("[S%d %d %d] ", int(s.swr), int(fwd)*10, int(rev)*10)
		//}

	}

}

func (s *TranscieverState) CreateSnapshotPacket() []byte {
	if s.snapshotPacketBuffer == nil {
		s.snapshotPacketBuffer = make([]byte, 300) // it's basically only few regs
	}
	// will produce standard packet to 28 th endpoint
	buf := s.snapshotPacketBuffer
	buf[0] = 0xEF
	buf[1] = 0xFE
	buf[2] = 28
	buf[3] = 6
	buf[4] = 0
	buf[5] = 0
	buf[6] = 0
	buf[7] = 0
	buf[8] = 0x7F
	buf[9] = 0x7F
	buf[10] = 0x7F
	dest := 11
	for ix, r := range s.knownregs {
		if r {
			buf[dest+0] = s.lowStateBits | uint8(ix<<3)
			buf[dest+1] = s.registers[ix][0]
			buf[dest+2] = s.registers[ix][1]
			buf[dest+3] = s.registers[ix][2]
			buf[dest+4] = s.registers[ix][3]
			dest += 5
		}
	}
	buf[dest+0] = 0xFF // end marker
	dest++
	return buf[:dest]
}

func (s *ClientState) updateStateFromUDP(ctm time.Time, udpData []byte) {
	if udpData[0] == 0xEF && udpData[1] == 0xFE {
		switch udpData[2] {
		case 1: // iq data from APP
			endpoint := udpData[3]
			// force update sequence. Keeping same sequence numbers, adding inserted (from proxy only -> TRX) packets.
			s.udpSendSequence = uint32(udpData[4])<<24 | uint32(udpData[5])<<16 | uint32(udpData[6])<<8 | uint32(udpData[7])
			insertions := s.udpInsertions
			udpData[4] = byte((s.udpSendSequence + insertions) >> 24)
			udpData[5] = byte((s.udpSendSequence + insertions) >> 16)
			udpData[6] = byte((s.udpSendSequence + insertions) >> 8)
			udpData[7] = byte(s.udpSendSequence + insertions)
			if endpoint == 2 {
				s.updateStateFromFrame(ctm, udpData[8:8+504])
				s.updateStateFromFrame(ctm, udpData[520:520+504])
				s.last83 = udpData[8+3] // ptt state etc
			}
		case 4: // start request from app
			logg.Printf("START/STOP stream request from APP: %d %d ( source: %v )", udpData[3]&0x01, (udpData[3]&0x02)>>1, s.addr)
			break
		}
	}
}

func (s *ClientState) updateStateFromFrame(ctm time.Time, frame []byte) {
	if frame[0] == 0x7F && frame[1] == 0x7F && frame[2] == 0x7F {
		newTransmit := frame[3]&0x01 == 1
		if newTransmit != s.transmit {
			logg.Println("\nTRANSMIT PTT FROM APP:", newTransmit)
			s.transmit = newTransmit
			s.transmitChangeTime = ctm
			if !newTransmit {
				s.transmitFramesSentToTRX = 0
			}
		}
		register := frame[3] >> 1
		s.knownregs[register] = true
		cin1 := frame[4]
		cin2 := frame[5]
		cin3 := frame[6]
		cin4 := frame[7]
		if s.registers[register][0] != cin1 ||
			s.registers[register][1] != cin2 ||
			s.registers[register][2] != cin3 ||
			s.registers[register][3] != cin4 {
			// changed
			s.registers[register][0] = cin1
			s.registers[register][1] = cin2
			s.registers[register][2] = cin3
			s.registers[register][3] = cin4

			changed := true

			if changed {
				logg.Printf("APP register: %x %x %x %x\n", register, frame[4], frame[5], frame[6])
			}
		}
	}
}

var buffers = make(chan []byte, 1000)

func obtainBuffer() []byte {
	select {
	case b := <-buffers:
		return b
	default:
		return make([]byte, 1200)
	}
}

func returnBuffer(b []byte) {
	if len(b) == 1200 {
		buffers <- b
	}
}

type Job struct {
	buffer   []byte
	n        int
	addr     netip.AddrPort
	recvTime time.Time
}

func recvfromInet4(fd int, p []byte, flags int) (n int, from netip.AddrPort, err error) {
	type _Socklen uint32
	var rsa syscall.Sockaddr
	//var socklen _Socklen = syscall.SizeofSockaddrAny
	n, rsa, err = syscall.Recvfrom(fd, p, flags)
	if err != nil {
		return
	}
	inet6, ok := rsa.(*syscall.SockaddrInet6)
	if ok {
		var a4 netip.Addr
		a4 = netip.AddrFrom4([4]byte{inet6.Addr[12], inet6.Addr[13], inet6.Addr[14], inet6.Addr[15]})
		from = netip.AddrPortFrom(a4, uint16(inet6.Port))
	} else {
		inet4, ok := rsa.(*syscall.SockaddrInet4)
		if ok {
			var a4 netip.Addr
			a4 = netip.AddrFrom4(inet4.Addr)
			from = netip.AddrPortFrom(a4, uint16(inet4.Port))
		} else {
			err = errors.New("cannot deduce an address after syscall recvfrom")
		}
	}
	return
}

var recvChan = make(chan Job, 1000) // receive -> process
var sendChan = make(chan Job, 1000) // process -> send

func receiveLoop() {
	// receiving loop
	for {
		var j Job
		var err error
		j.buffer = obtainBuffer()
		j.n, j.addr, err = frontend.ReadFromUDPAddrPort(j.buffer)
		j.recvTime = time.Now()
		if err != nil {
			logg.Fatal(err)
		}
		recvChan <- j
	}
}

func receiveLoopTCPDump() {

	err2 := exec.Command("sh", "-c", "killall -9 tcpdump; pkill -9 tcpdump; test -r dump.pipe || mkfifo dump.pipe").Run()
	if err2 != nil {
		fmt.Println("Mkfifo:", err2)
		return
	}
	time.Sleep(500 * time.Millisecond)

	cmd := exec.Command("tcpdump", "-s", "65535", "-U", "-w", "dump.pipe", "udp")

	// Get the pipe to the standard output of the command
	stderr, err := cmd.StderrPipe()
	//stdout, err := cmd.StdoutPipe()
	if err != nil {
		fmt.Println("Error getting stdout:", err)
		return
	}

	// Start the command
	if err := cmd.Start(); err != nil {
		fmt.Println("Error starting command:", err)
		return
	}

	go func() {
		reader := bufio.NewReader(stderr)
		for {
			b, errx := reader.ReadByte()
			if errx != nil {
				break
			}
			fmt.Printf("%c", b)
		}
	}()

	// Create a buffer to read chunks from stdout
	packetInput, err2 := os.Open("dump.pipe")

	//packetInput := stdout
	if err2 != nil {
		fmt.Println("Error opening pipe:", err)
		return
	}

	reader := packetInput // bufio.NewReaderSize(packetInput, 24)
	logg.Println("Launched tcpdump, waiting for data..." + runtime.GOARCH + " " + runtime.GOOS + " " + runtime.Version())
	var packetSource *gopacket.PacketSource
	if runtime.GOOS == "darwin" {
		logg.Println("Using PCAPNG")
		newReader, err := pcapgo.NewNgReader(reader, pcapgo.NgReaderOptions{})
		if err != nil {
			panic(err)
		}
		packetSource = gopacket.NewPacketSource(newReader, gopacket.DecodePayload)
	} else {
		logg.Println("Using PCAP")
		newReader, err := pcapgo.NewReader(reader)
		if err != nil {
			panic(err)
		}
		packetSource = gopacket.NewPacketSource(newReader, gopacket.DecodePayload)
	}
	for packet := range packetSource.Packets() {
		data := packet.Data()
		if len(data) < 28 {
			logg.Println("Small packet: ", len(data))
			continue
		}
		if data[0xC] == 0x08 && data[0x17] == 0x11 { // ipv4, udp
			sourcePort := (uint16(data[0x24]) << 8) + uint16(data[0x25])
			destPort := (uint16(data[0x22]) << 8) + uint16(data[0x23])
			sourceAddrSlice := data[0x1A : 0x1A+4]
			destAddrSlice := data[0x1E : 0x1E+4]
			if (sourcePort == 1024) || (destPort == 1024) { // src or dest port == 1024
				if destAddrSlice[0] == 255 {
					// broadcast
					continue
				}
				//log.Println("packet: src", sourceAddrSlice, "dst", destAddrSlice)
				if isGatewayIP(sourceAddrSlice) || !isGatewayIP(destAddrSlice) {
					continue
				}
				addr, _ := netip.AddrFromSlice(sourceAddrSlice)
				srcAddress := netip.AddrPortFrom(addr, uint16(data[0x22])<<8+uint16(data[0x23]))

				payloadLen := uint16(data[0x27]) + (uint16(data[0x26]) << 8) - 8 // payload length
				payload := data[0x2A : 0x2A+payloadLen]

				// Print source IP, source port, and payload
				//fmt.Printf("Source IP: %s, Source Port: %d\n", ipv4.SrcIP, udp.SrcPort)
				//fmt.Printf("Payload: %x\n", udp.Payload)
				var j Job
				j.n = int(payloadLen)
				j.buffer = payload
				j.addr = srcAddress
				j.recvTime = packet.Metadata().CaptureInfo.Timestamp

				//logg.Println("recvloop: Received from ", j.addr)

				recvChan <- j
			} else {
				//logg.Println("recvloop: tcpdump: other ports in packet:", sourceAddrSlice, sourcePort, destAddrSlice, destPort)
			}
		} else {
			//logg.Println("recvloop: tcpdump: non udp/tcp4: ")
			//logg.Println(packet.String())
		}
		// Process packet here
		//fmt.Println(packet)
		//count++
		//if pkt.Metadata().CaptureInfo.Timestamp.Sub(prevReport) > 1*time.Second {
		//	logg.Println("Processed", count, "packets/sec")
		//	count = 0
		//	prevReport = pkt.Metadata().CaptureInfo.Timestamp
		//}

	}

	const BufSize = 1024 // Size of the chunk to read (in bytes)
	buf := make([]byte, BufSize)

	for {
		// Read chunk into buffer
		n, err := reader.Read(buf)

		if n > 0 {
			// Process the chunk
			fmt.Print(string(buf[:n]))
		}

		if err == io.EOF {
			break
		} else if err != nil {
			fmt.Println("Error reading from stdout:", err)
			break
		}
	}

	// Wait for the command to finish
	if err := cmd.Wait(); err != nil {
		fmt.Println("Command finished with error:", err)
	}
}

func isGatewayIP(ip []byte) bool {
	return ip[0] == 192 && ip[1] == 168 && ip[2] == 8 && ip[3] == 1
}

func sendLoop() {
	// transmitting loop

	lastSend := time.Now()
	outCount := 0
	for {
		var j Job
		var err error
		j = <-sendChan
		//logg.Println("sendloop: Sending to ", j.addr, len(j.buffer))
		if j.buffer == nil {
			log.Fatal("nil buffer to send")
		}
		if isSameAddress(j.addr, backendAddr) {
			_, err = backendConn.Write(j.buffer[:j.n]) // simulate to trx
		} else {
			_, err = frontend.Write(j.buffer[:j.n]) // simulate to trx
		}
		if err != nil {
			logg.Println("Send error: ", err)
		}
		ctm := time.Now()
		dt := ctm.Sub(lastSend)
		drt := ctm.Sub(j.recvTime).Milliseconds()
		if j.recvTime.IsZero() {
			drt = 0
		}
		lastSend = ctm
		if isSameAddress(j.addr, backendAddr) {
			if dt > 8*time.Millisecond || drt > 12 {
				logg.Println("Slow send/process: ", outCount, dt.Milliseconds(), drt)
			}
		}
		returnBuffer(j.buffer)
		outCount++
	}
}

func sendToTrx(packet []byte, n int, recvTime time.Time) {
	sendChan <- Job{packet, n, backendAddr, recvTime}
	clientState.transmitFramesSentToTRX++
	transcieverState.sentSinceFillLevelTime++
}

func handleFrontend(backend *net.UDPAddr, wg *sync.WaitGroup) {
	defer wg.Done()

	if *useTcpdump {
		go receiveLoopTCPDump()
	}
	//go receiveLoop(frontend)		// the default socket will receive data from trx, because we use it as outgoing towards TRX
	go sendLoop()
	go func() {
		for {
			j := <-recvChan
			ctm := time.Now()

			ctmMilli := ctm.UnixMilli()
			//log.Println("recvd from: ", j.addr)
			if isSameAddress(j.addr, backendAddr) {
				// message from backend (trx)
				if clientState.addr.Port() != 0 {
					handleTRXPacket(ctm, j)
					transcieverState.sendPackets = nil
					if ctmMilli-lastServerReport > 1000 {
						//logg.Printf("[%d] Sent message (%d) to client %s\n", serverCount, j.n, clientState.addr.String())
						lastServerReport = ctmMilli
					}
					serverCount++
				}
			} else {
				// message from software
				if !isSameAddress(clientState.addr, j.addr) {
					clientState = ClientState{}
					clientState.addr = j.addr
					logg.Println("new clientstate:", j.addr)
					talkback, _ := net.ResolveUDPAddr("udp", j.addr.String())
					frontend, _ = net.DialUDP("udp", frontendAddr, talkback)

					if *doSuspendClientTX {
						go func() {
							time.Sleep(5 * time.Second)
							APPTXsuspended = true
						}()
					}
					if *doCpuProfile {
						*doCpuProfile = false
						cpuProfile, err := os.Create("cpu.pprof")
						if err != nil {
							panic(err)
						}
						if err := pprof.StartCPUProfile(cpuProfile); err != nil {
							panic(err)
						}
						go func() {
							time.Sleep(5 * time.Second)
							pprof.StopCPUProfile()
							cpuProfile.Close()
							log.Println("ending cpu profile.")
							os.Exit(0)
						}()
					}
				}
				clientState.updateStateFromUDP(ctm, j.buffer[:j.n])
				clientCount++
				sendToTrx(j.buffer, j.n, j.recvTime)
				//dt := ctm.Sub(clientState.lastReceived)
				clientState.lastReceived = ctm
				clientState.lastTrueReceived = ctm
				if ctmMilli-lastClientReport > 1000 || (clientState.transmit && *verboseTxTiming) {
					//logg.Printf("[%d] dt=%v tx=%v Relayed message (len=%d) from APP to TRX  %s\n", clientCount, dt, clientState.transmit, j.n, j.addr.String())
					lastClientReport = ctmMilli
				}
			}

			maybeSimulatePacketToRRX(ctm)

		}
	}()

}

func maybeSimulatePacketToRRX(ctm time.Time) {
	verboseSimu := true
	timeToSimulateFromAPP := ctm.Sub(clientState.lastReceived) > 300*time.Millisecond
	if clientState.transmit {
		fl := transcieverState.getFillLevel(ctm)
		if fl < *fillLevel && fl >= 0 && *fillLevel > 0 {
			timeToSimulateFromAPP = true
			//logg.Println("filling missing packet: queue len = ", transcieverState.fillLevel, "(", transcieverState.getFillLevel(ctm), ")", " since ", ctm.Sub(transcieverState.fillLevelTime))
			logg.PrintfStd("[T%d] ", fl)
			verboseSimu = false
		}
	}

	if *fillLevel == 0 && clientState.transmit {
		//logg.PrintfStd("[Q%d %d] ", transcieverState.fillLevel, transcieverState.getFillLevel(ctm))
	}

	if timeToSimulateFromAPP && ctm.Sub(clientState.lastTrueReceived) < 3000*time.Millisecond {
		// 3 seconds without client data = OK, otherwise dont send anything. TRX will stop stream eventually.
		simulatedPacket := obtainBuffer()
		// client does not send anything. Maybe it knows we're hl2 proxy
		// maybe nothing changed.
		clientState.udpInsertions++
		newSeq := clientState.udpSendSequence + clientState.udpInsertions
		simulatedPacket[0] = 0xEF // dest endpoint
		simulatedPacket[1] = 0xFE // dest endpoint
		simulatedPacket[2] = 1    // iq data from APP
		simulatedPacket[3] = 0x02 // dest endpoint
		simulatedPacket[4] = byte(newSeq >> 24)
		simulatedPacket[5] = byte(newSeq >> 16)
		simulatedPacket[6] = byte(newSeq >> 8)
		simulatedPacket[7] = byte(newSeq)
		simulatedPacket[8] = 0x7F
		simulatedPacket[9] = 0x7F
		simulatedPacket[10] = 0x7F

		simulatedPacket[8+3] = byte(clientState.lastSentRegister<<3) | (clientState.last83 & 0x7)
		simulatedPacket[8+4] = clientState.registers[clientState.lastSentRegister][0]
		simulatedPacket[8+5] = clientState.registers[clientState.lastSentRegister][1]
		simulatedPacket[8+6] = clientState.registers[clientState.lastSentRegister][2]
		simulatedPacket[8+7] = clientState.registers[clientState.lastSentRegister][3]

		clientState.lastSentRegister++
		// fill register
		for ; clientState.lastSentRegister < 255 && !clientState.knownregs[clientState.lastSentRegister]; clientState.lastSentRegister++ {
		}
		if clientState.lastSentRegister >= 255 {
			clientState.lastSentRegister = 0
		}

		// fill in registers, in loop
		simulatedPacket[520+0] = 0x7F
		simulatedPacket[520+1] = 0x7F
		simulatedPacket[520+2] = 0x7F
		simulatedPacket[520+3] = byte(clientState.lastSentRegister<<3) | (clientState.last83 & 0x7)
		simulatedPacket[520+4] = clientState.registers[clientState.lastSentRegister][0]
		simulatedPacket[520+5] = clientState.registers[clientState.lastSentRegister][1]
		simulatedPacket[520+6] = clientState.registers[clientState.lastSentRegister][2]
		simulatedPacket[520+7] = clientState.registers[clientState.lastSentRegister][3]

		sendToTrx(simulatedPacket, 1032, time.Time{}) // std pkt size
		clientState.lastReceived = ctm                // kinda received
		if verboseSimu {
			//logg.Println("Simulated client packet, newseq=", newSeq, " register=", clientState.lastSentRegister, ": error?=", err)
			//logg.PrintfStd("[A] ")
		}
	}

}

func handleTRXPacket(ctm time.Time, j Job) {
	transcieverState.updateStateFromUDP(ctm, j.buffer[:j.n])
	// reply to incoming addr (client).
	if clientState.transmit {
		// don't send anything to the APP while transmitting
		if ctm.Sub(clientState.lastSentTowards) > 250*time.Millisecond {
			snapshotPacket := transcieverState.CreateSnapshotPacket()
			//logg.Println("Sent snapshot packet towards APP, len=", len(snapshotPacket))
			if !APPTXsuspended {
				sendChan <- Job{snapshotPacket, len(snapshotPacket), clientState.addr, j.recvTime}
			}
			clientState.lastSentTowards = ctm
		}
	} else {
		if !APPTXsuspended {
			//logg.Println("processor: will send to: ", clientState.addr)
			sendChan <- Job{j.buffer, j.n, clientState.addr, j.recvTime}
		}
		//_, err = frontend.WriteToUDP(buffer[:n], &clientState.addr)
	}
	for _, pkt := range transcieverState.sendPackets {
		if !APPTXsuspended {
			sendChan <- Job{pkt, len(pkt), clientState.addr, j.recvTime}
		}
		//_, err = frontend.WriteToUDP(pkt, &clientState.addr)
	}

}

func isSameAddress(addr netip.AddrPort, addr2 netip.AddrPort) bool {
	if addr.Port() != addr2.Port() {
		return false
	}
	if addr.Addr().Is4() && addr2.Addr().Is4() {
		return addr.Addr().Compare(addr2.Addr()) == 0
	}
	return addr.Addr().As4() == addr2.Addr().As4()
}

var verboseTxTiming = flag.Bool("verbosetx", false, "Verbose log tx packets timings")
var fillLevel = flag.Int("fill", 4, "fill level to start simulate filling")
var doCpuProfile = flag.Bool("prof", false, "cpu profile few seconds of TRX->APP traffic")
var doSuspendClientTX = flag.Bool("testsct", false, "test suspend clent tx")
var doRawTest = flag.Bool("rawtest", false, "raw socket test")
var doPCAPTest = flag.Bool("pcap", false, "pcap test")
var useTcpdump = flag.Bool("usedump", true, "use tcpdump for receive")

func runLogger(logg *MyLog) {
	lastTime := time.Now()
	for {
		ch := <-logg.pipe
		if ch.when.IsZero() {
			str := ch.what
			fmt.Print(str)
		} else {
			pref := ""
			if strings.HasPrefix(ch.what, "\n") {
				pref = "\n"
				ch.what = ch.what[1:]
			}
			str := fmt.Sprintf("%s%02d:%02d:%02d.%03d (+%0.3f) %s",
				pref,
				ch.when.Hour(), ch.when.Minute(), ch.when.Second(),
				ch.when.Nanosecond()/1000000, float64(ch.when.Sub(lastTime).Nanoseconds())/1000000.0, ch.what)
			fmt.Print(str)
			lastTime = ch.when
		}
		if ch.fatal {
			os.Exit(1)
		}
	}
}

func main() {
	// C.ctest()
	//log.Println("C loop exited..")

	logg.pipe = make(chan LogMessage, 10000)
	go runLogger(&logg)
	go func() {
		for {
			prevServer, prevClient := serverCount, clientCount
			time.Sleep(1 * time.Second)
			logg.Println("\nPacket counts/1sec received: from TRX:", serverCount-prevServer, "  from APP:", clientCount-prevClient)
		}
	}()
	serverAddress := flag.String("hermes", "192.168.8.130:1024", "Hermes server address")
	clientAddress := flag.String("bind", ":1024", "Listen port for client connection")
	flag.Parse()
	if !flag.Parsed() {
		flag.Usage()
		logg.Fatal("ERROR: unable to parse flags")
	}
	if *doRawTest {
		go doRawSocketText()
	}
	if *doPCAPTest {
		pcapTest()
	}

	var wg sync.WaitGroup

	// Resolve server and client addresses
	backendAddrx, err := net.ResolveUDPAddr("udp", *serverAddress)
	if err != nil {
		logg.Fatal(err)
	}
	backendAddr = backendAddrx.AddrPort()
	backendAddr = netip.MustParseAddrPort(*serverAddress)

	frontendAddr, err = net.ResolveUDPAddr("udp", *clientAddress)
	if err != nil {
		logg.Fatal(err)
	}

	if *useTcpdump {
		logg.Println("Will listen using tcpdump (capture all)")
	} else {
		logg.Println("Listening on frontend " + frontendAddr.String() + " (wanted: " + *clientAddress + ")")
	}

	backendConn, err = net.DialUDP("udp", nil, backendAddrx)
	if err != nil {
		logg.Fatal(err)
	}
	defer backendConn.Close()

	logg.Println("Proxy started...")

	// Handle incoming connections in separate goroutines
	wg.Add(2)

	go handleFrontend(backendAddrx, &wg)

	wg.Wait()
}

func pcapTest() {
	//if handle, err := pcap.OpenLive("br-lan", 1600, true, pcap.BlockForever); err != nil {
	//	panic(err)
	//} else if err := handle.SetBPFFilter("udp"); err != nil { // optional
	//	panic(err)
	//} else {
	//	packetSource := gopacket.NewPacketSource(handle, handle.LinkType())
	//	for packet := range packetSource.Packets() {
	//		log.Println(packet)
	//		//handlePacket(packet)  // Do something with a packet here.
	//	}
	//}
}

func doRawSocketText() {
	conn, err := net.ListenPacket("ip4:udp", "0.0.0.0") // Listening for ICMP for demonstration
	if err != nil {
		fmt.Println("Error opening raw socket:", err)
		os.Exit(1)
	}
	defer conn.Close()

	// Buffer to read into
	buffer := make([]byte, 1024000)

	// Channel to signal every second
	ticker := time.NewTicker(time.Second)
	defer ticker.Stop()

	byteCount := 0
	packetCount := 0

	// Go routine to read from socket
	go func() {
		for {
			n, _, err := conn.ReadFrom(buffer)
			if err != nil {
				fmt.Println("Error reading from raw socket:", err)
				continue
			}
			// Process n bytes here
			//fmt.Printf("Received %d bytes\n", n)
			packetCount++
			byteCount += n
		}
	}()

	// Main loop
	for {
		select {
		case <-ticker.C:
			// Reset or display byte count here
			fmt.Println("RAWSOCKET: One second passed: ", packetCount, byteCount)
		}
	}
}
