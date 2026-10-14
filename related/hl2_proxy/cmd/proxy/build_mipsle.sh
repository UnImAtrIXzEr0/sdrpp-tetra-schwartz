GOOS=linux GOARCH=mipsle go build && (
scp -O proxy 192.168.8.1:
ssh 192.168.8.1 ash -c "export GODEBUG=gctrace=1; ./proxy -fill 0"
scp -O 192.168.8.1:cpu.pprof .
)
