package main

import (
	"bytes"
	"fmt"
	"io"
	"os"

	"github.com/gorilla/websocket"
)

import (
	"crypto/md5"
	"time"
)

const (
	separator = "----------sdr++--payload---separator-----"
)

func generateSecretKey() string {
	now := time.Now().Unix()
	window := now / 3600
	keyBase := fmt.Sprintf("%dSDR++Brown FTW", window)
	hash := fmt.Sprintf("%x", md5.Sum([]byte(keyBase)))
	fmt.Printf("Generating secret key:\n")
	fmt.Printf("  Current time: %d\n", now)
	fmt.Printf("  Window: %d\n", window)
	fmt.Printf("  Key base: %s\n", keyBase)
	fmt.Printf("  MD5 hash: %s\n", hash)
	return hash
}

type Response struct {
	Command string `json:"command"`
	Error   string `json:"error,omitempty"`
}

func main() {
	if len(os.Args) < 2 {
		fmt.Println("Usage: service_client <audio_file>")
		return
	}

	filename := os.Args[1]
	file, err := os.Open(filename)
	if err != nil {
		fmt.Printf("Error opening file: %v\n", err)
		return
	}
	defer file.Close()

	// Read the entire file into memory
	audioData, err := io.ReadAll(file)
	if err != nil {
		fmt.Printf("Error reading file: %v\n", err)
		return
	}

	// Create the message
	var msg bytes.Buffer
	
	// Write secret key
	secret := generateSecretKey()
	msg.WriteString(secret)
	
	// Write separator
	msg.WriteString(separator)
	
	// Write audio data
	msg.Write(audioData)
	
	// Debug print the secret key being sent
	fmt.Printf("Sending secret key: %s\n", secret)

	// Create WebSocket connection
	conn, _, err := websocket.DefaultDialer.Dial("ws://brownai.san.systems:8080/ws", nil)
	if err != nil {
		fmt.Printf("Error connecting to WebSocket: %v\n", err)
		return
	}
	defer conn.Close()

	// Send message
	if err := conn.WriteMessage(websocket.BinaryMessage, msg.Bytes()); err != nil {
		fmt.Printf("Error sending message: %v\n", err)
		return
	}

	// Read response
	var result Response
	if err := conn.ReadJSON(&result); err != nil {
		fmt.Printf("Error reading response: %v\n", err)
		return
	}

	if result.Error != "" {
		fmt.Printf("Error: %s\n", result.Error)
	} else {
		fmt.Printf("Detected command: %s\n", result.Command)
	}
}
