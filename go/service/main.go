package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"mime/multipart"
	"net/http"
	"os"
	"strings"
	"sync"

	"github.com/gorilla/websocket"
)

import (
	"crypto/md5"
	"time"
)

const (
	separator = "----------sdr++--payload---separator-----"
)

func generateSecretKey(t int64) string {
	window := t / 3600
	keyBase := fmt.Sprintf("%dSDR++Brown FTW", window)
	hash := fmt.Sprintf("%x", md5.Sum([]byte(keyBase)))
	fmt.Printf("Generating secret key:\n")
	fmt.Printf("  Timestamp: %d\n", t)
	fmt.Printf("  Window: %d\n", window)
	fmt.Printf("  Key base: %s\n", keyBase)
	fmt.Printf("  MD5 hash: %s\n", hash)
	return hash
}

var groqAPIKey string

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

type WhisperRequest struct {
	File           []byte `json:"file"`
	Model          string `json:"model"`
	Temperature    string `json:"temperature"`
	ResponseFormat string `json:"response_format"`
	Language       string `json:"language"`
}

type LLMRequest struct {
	Model    string        `json:"model"`
	Messages []LLMMessage `json:"messages"`
}

type LLMMessage struct {
	Role    string `json:"role"`
	Content string `json:"content"`
}

type Response struct {
	Command string `json:"command"`
	Error   string `json:"error,omitempty"`
}

var whisperClient = &http.Client{}
var llmClient = &http.Client{}
var requestMutex sync.Mutex

func main() {
	// Get API key from environment
	groqAPIKey = os.Getenv("GROQ_API_KEY")
	if groqAPIKey == "" {
		fmt.Println("Error: GROQ_API_KEY environment variable is required")
		os.Exit(1)
	}

	http.HandleFunc("/ws", handleWebSocket)
	fmt.Println("Starting server on :8080...")
	if err := http.ListenAndServe(":8080", nil); err != nil {
		fmt.Println("Error starting server:", err)
		os.Exit(1)
	}
}

func handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		fmt.Println("WebSocket upgrade error:", err)
		return
	}
	defer conn.Close()

	for {
		_, message, err := conn.ReadMessage()
		if err != nil {
			fmt.Println("Read error:", err)
			break
		}

		// Process the binary message
		response := processMessage(message)
		
		// Send response back
		if err := conn.WriteJSON(response); err != nil {
			fmt.Println("Write error:", err)
			break
		}
	}
}

func processMessage(message []byte) Response {
	// Split message using separator
	parts := bytes.Split(message, []byte(separator))
	if len(parts) != 2 {
		return Response{Error: "invalid message format"}
	}

	// Read secret key
	receivedKey := string(parts[0])
	
	// Get current time and calculate window
	now := time.Now().Unix()
	currentWindow := now / 3600
	
	// Check current window and ±1
	valid := false
	fmt.Printf("Validating received key: %s\n", receivedKey)
	for i := -1; i <= 1; i++ {
		window := currentWindow + int64(i)
		fmt.Printf("Checking window %d:\n", window)
		expectedKey := generateSecretKey(window * 3600)
		if receivedKey == expectedKey {
			fmt.Printf("Key match found in window %d\n", window)
			valid = true
			break
		}
		fmt.Printf("Key did not match for window %d\n", window)
	}
	
	if !valid {
		return Response{Error: "invalid secret key"}
	}

	// Process audio
	audioData := parts[1]
	whisperResult, err := callWhisper(audioData)
	if err != nil {
		fmt.Printf("Whisper error: %v\n", err)
		return Response{Error: fmt.Sprintf("Whisper error: %v", err)}
	}
	fmt.Printf("Whisper result: %s\n", whisperResult)

	// Process LLM
	command, err := callLLM(whisperResult)
	if err != nil {
		fmt.Printf("LLM error: %v\n", err)
		return Response{Error: fmt.Sprintf("LLM error: %v", err)}
	}
	fmt.Printf("LLM command: %s\n", command)

	return Response{Command: command}
}

func callWhisper(audioData []byte) (string, error) {
	requestMutex.Lock()
	defer requestMutex.Unlock()

	startTime := time.Now()
	fileSize := len(audioData)
	fmt.Printf("Calling Whisper API with audio file size: %d bytes\n", fileSize)

	// Create multipart form data
	body := &bytes.Buffer{}
	writer := multipart.NewWriter(body)
	
	part, err := writer.CreateFormFile("file", "audio.wav")
	if err != nil {
		return "", err
	}
	if _, err := io.Copy(part, bytes.NewReader(audioData)); err != nil {
		return "", err
	}

	writer.WriteField("model", "whisper-large-v3")
	writer.WriteField("temperature", "0")
	writer.WriteField("response_format", "json")
	writer.WriteField("language", "en")
	writer.Close()

	req, err := http.NewRequest("POST", "https://api.groq.com/openai/v1/audio/transcriptions", body)
	if err != nil {
		return "", err
	}
	req.Header.Set("Content-Type", writer.FormDataContentType())
	req.Header.Set("Authorization", "Bearer " + groqAPIKey)

	resp, err := whisperClient.Do(req)
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return "", fmt.Errorf("whisper API returned status %d", resp.StatusCode)
	}

	duration := time.Since(startTime)
	fmt.Printf("Whisper API call completed in %v\n", duration)

	var result map[string]interface{}
	if err := json.NewDecoder(resp.Body).Decode(&result); err != nil {
		return "", err
	}

	return result["text"].(string), nil
}

func callLLM(whisperResult string) (string, error) {
	requestMutex.Lock()
	defer requestMutex.Unlock()

	startTime := time.Now()
	fmt.Printf("Calling LLM API with whisper result length: %d characters\n", len(whisperResult))

	prompt := `You are the assistant who helps to understand people with dislexia. You will be given badly spelt command
               and you must understand it and narrow it down to listed commands below.

               # Commands and their context

               - anything related to "upper" - meaining upper side band - COMMAND: USB
               - anything related to "lower" - meaining lower side band - COMMAND: LSB
               - anything related to amplitude - meaining amplitude modulation - COMMAND: AM
               - anything related to frequency modulation - meaining frequency modulation - COMMAND: FM
               - any direct number input, with decimal point or without, with kilohertz or megahertz suffix - meaning change frequency - COMMAND: FREQ number suffix
               - request to lower volume - COMMAND: VOLUME_DOWN
               - request to make sound louder - COMMAND: VOLUME_UP
               - request to mute, unmute, or stop or resume sound - COMMAND: MUTE
               - pause/stop play, or just pause/stop - COMMAND: STOP
               - play/launch/go/start - COMMAND: START
               - everything else - COMMAND: UNKNOWN;

               EXAMPLES:

               User input: run this thing
               Assistant: COMMAND: START

               User input: one three three point 7
               Assistant: COMMAND: FREQ 133.7 nothing

               User input: sixteen hundred kilohertz
               Assistant: COMMAND: FREQ 1600 KHz

               User input: one hundred 44 megahertz point 2
               Assistant: COMMAND: FREQ 144.2 MHz

               User input: A M
               Assistant: COMMAND: AM

               User input: i like you
               Assistant: COMMAND: UNKNOWN

               You must not output anything else than COMMAND keyword and then command itself, and you must output command itself in capital letters.
`

	llmRequest := LLMRequest{
		Model: "llama-3.3-70b-versatile",
		Messages: []LLMMessage{
			{Role: "system", Content: prompt},
			{Role: "user", Content: "# User input\n" + whisperResult},
		},
	}

	reqBody, err := json.Marshal(llmRequest)
	if err != nil {
		return "", err
	}

	req, err := http.NewRequest("POST", "https://api.groq.com/openai/v1/chat/completions", bytes.NewBuffer(reqBody))
	if err != nil {
		return "", err
	}
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Authorization", "Bearer " + groqAPIKey)

	resp, err := llmClient.Do(req)
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		return "", fmt.Errorf("LLM API returned status %d", resp.StatusCode)
	}

	duration := time.Since(startTime)
	fmt.Printf("LLM API call completed in %v\n", duration)

	var result map[string]interface{}
	if err := json.NewDecoder(resp.Body).Decode(&result); err != nil {
		return "", err
	}

	// Extract command from response
	choices := result["choices"].([]interface{})
	if len(choices) == 0 {
		return "", fmt.Errorf("no choices in LLM response")
	}
	message := choices[0].(map[string]interface{})["message"].(map[string]interface{})
	content := message["content"].(string)

	// Log the full LLM response for debugging
	fmt.Printf("Full LLM response:\n%s\n", content)

	// Extract command after last "COMMAND" line
	lines := strings.Split(content, "\n")
	for i := len(lines) - 1; i >= 0; i-- {
		if strings.Contains(lines[i], "COMMAND") {
			cmd := strings.TrimSpace(strings.SplitN(lines[i], "COMMAND", 2)[1])
			fmt.Printf("Extracted command: %s\n", cmd)
			return cmd, nil
		}
	}

	fmt.Printf("No COMMAND found in LLM response. Last 3 lines:\n")
	for i := len(lines) - 3; i < len(lines); i++ {
		if i >= 0 {
			fmt.Printf("%s\n", lines[i])
		}
	}
	return "", fmt.Errorf("no command found in LLM response")
}
