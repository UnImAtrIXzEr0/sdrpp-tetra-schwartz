#pragma once
// NMEA GPS receiver on a serial port (USB GPS mice show up as /dev/ttyACM*,
// /dev/ttyUSB* on Linux and COMx on Windows). Read-only: the port is opened
// in its own thread, nothing is written to the device.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

struct GpsFix {
    bool connected = false;  // port open and NMEA sentences arriving
    bool valid = false;      // position fix (RMC status A / GGA quality > 0), not stale
    double lat = 0.0, lon = 0.0;
    double altM = 0.0;
    double speedKmh = 0.0;
    double courseDeg = 0.0;
    int sats = 0;
    double hdop = 0.0;
    int quality = 0;         // GGA fix quality
    std::string utc;         // hhmmss from the receiver
    double ageS = 1e9;       // seconds since the last valid fix
};

class NmeaGps {
public:
    ~NmeaGps() { stop(); }

    static std::vector<std::string> listPorts() {
        std::vector<std::string> ports;
#ifdef _WIN32
        char target[512];
        for (int i = 1; i <= 64; i++) {
            std::string name = "COM" + std::to_string(i);
            if (QueryDosDeviceA(name.c_str(), target, sizeof(target))) { ports.push_back(name); }
        }
#else
        const char* dirs[] = { "/dev/serial/by-id", "/dev" };
        for (const char* d : dirs) {
            DIR* dir = opendir(d);
            if (!dir) { continue; }
            struct dirent* e;
            while ((e = readdir(dir)) != nullptr) {
                std::string n = e->d_name;
                bool want = (std::string(d) == "/dev/serial/by-id") ? (n[0] != '.')
                          : (n.rfind("ttyACM", 0) == 0 || n.rfind("ttyUSB", 0) == 0);
                if (want) { ports.push_back(std::string(d) + "/" + n); }
            }
            closedir(dir);
        }
#endif
        return ports;
    }

    bool start(const std::string& port, int baud) {
        stop();
        {
            std::lock_guard<std::mutex> lck(mtx);
            error.clear();
            fix = GpsFix();
            lastSentence = std::chrono::steady_clock::time_point();
            lastValid = std::chrono::steady_clock::time_point();
        }
        if (!openPort(port, baud)) { return false; }
        running = true;
        worker = std::thread(&NmeaGps::run, this);
        return true;
    }

    void stop() {
        running = false;
        if (worker.joinable()) { worker.join(); }
        closePort();
    }

    bool isRunning() { return running.load(); }

    std::string getError() {
        std::lock_guard<std::mutex> lck(mtx);
        return error;
    }

    GpsFix get() {
        std::lock_guard<std::mutex> lck(mtx);
        GpsFix f = fix;
        auto now = std::chrono::steady_clock::now();
        f.connected = running && lastSentence.time_since_epoch().count() != 0 &&
                      std::chrono::duration<double>(now - lastSentence).count() < 3.0;
        if (lastValid.time_since_epoch().count() != 0) {
            f.ageS = std::chrono::duration<double>(now - lastValid).count();
        }
        f.valid = f.valid && f.ageS < 3.0;
        return f;
    }

    // ---- parsing (public for tests) ----
    void feed(const char* data, size_t len) {
        for (size_t i = 0; i < len; i++) {
            char c = data[i];
            if (c == '\n' || c == '\r') {
                if (!line.empty()) { parseLine(line); }
                line.clear();
            }
            else if (line.size() < 120) {
                line += c;
            }
            else {
                line.clear(); // garbage, resync on next line end
            }
        }
    }

private:
    static double nmeaCoord(const std::string& v, const std::string& hemi) {
        if (v.empty()) { return NAN; }
        double raw = atof(v.c_str());
        int deg = (int)(raw / 100.0);
        double d = deg + (raw - deg * 100.0) / 60.0;
        if (hemi == "S" || hemi == "W") { d = -d; }
        return d;
    }

    static bool checksumOk(const std::string& s) {
        size_t star = s.find('*');
        if (s.empty() || s[0] != '$') { return false; }
        if (star == std::string::npos) { return true; } // no checksum given
        if (star + 3 > s.size()) { return false; }
        unsigned char cs = 0;
        for (size_t i = 1; i < star; i++) { cs ^= (unsigned char)s[i]; }
        return (unsigned)strtoul(s.substr(star + 1, 2).c_str(), nullptr, 16) == cs;
    }

    void parseLine(const std::string& s) {
        if (!checksumOk(s)) { return; }
        std::string body = s.substr(1, s.find('*') == std::string::npos ? std::string::npos : s.find('*') - 1);
        std::vector<std::string> f;
        size_t start = 0;
        while (true) {
            size_t p = body.find(',', start);
            f.push_back(body.substr(start, p == std::string::npos ? std::string::npos : p - start));
            if (p == std::string::npos) { break; }
            start = p + 1;
        }
        if (f.empty() || f[0].size() < 5) { return; }
        std::string type = f[0].substr(2); // GPRMC, GNRMC, ... -> RMC
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lck(mtx);
        lastSentence = now;
        if (type == "RMC" && f.size() >= 9) {
            fix.utc = f[1].substr(0, 6);
            bool active = (f[2] == "A");
            double lat = nmeaCoord(f[3], f[4]);
            double lon = nmeaCoord(f[5], f[6]);
            if (active && !std::isnan(lat) && !std::isnan(lon)) {
                fix.lat = lat;
                fix.lon = lon;
                fix.speedKmh = atof(f[7].c_str()) * 1.852;
                fix.courseDeg = atof(f[8].c_str());
                fix.valid = true;
                lastValid = now;
            }
            else {
                fix.valid = false;
            }
        }
        else if (type == "GGA" && f.size() >= 10) {
            fix.quality = atoi(f[6].c_str());
            fix.sats = atoi(f[7].c_str());
            fix.hdop = atof(f[8].c_str());
            fix.altM = atof(f[9].c_str());
            double lat = nmeaCoord(f[2], f[3]);
            double lon = nmeaCoord(f[4], f[5]);
            if (fix.quality > 0 && !std::isnan(lat) && !std::isnan(lon)) {
                fix.lat = lat;
                fix.lon = lon;
                fix.valid = true;
                lastValid = now;
            }
        }
    }

    void setError(const std::string& e) {
        std::lock_guard<std::mutex> lck(mtx);
        error = e;
    }

#ifdef _WIN32
    HANDLE h = INVALID_HANDLE_VALUE;

    bool openPort(const std::string& port, int baud) {
        std::string path = "\\\\.\\" + port;
        h = CreateFileA(path.c_str(), GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (h == INVALID_HANDLE_VALUE) {
            setError("Cannot open " + port + " (in use or not present)");
            return false;
        }
        DCB dcb;
        ZeroMemory(&dcb, sizeof(dcb));
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h, &dcb);
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        SetCommState(h, &dcb);
        COMMTIMEOUTS to;
        ZeroMemory(&to, sizeof(to));
        to.ReadIntervalTimeout = 50;
        to.ReadTotalTimeoutConstant = 200;
        SetCommTimeouts(h, &to);
        return true;
    }

    int readSome(char* buf, int n) {
        DWORD got = 0;
        if (!ReadFile(h, buf, n, &got, NULL)) { return -1; }
        return (int)got;
    }

    void closePort() {
        if (h != INVALID_HANDLE_VALUE) {
            CloseHandle(h);
            h = INVALID_HANDLE_VALUE;
        }
    }
#else
    int fd = -1;

    static speed_t baudConst(int b) {
        switch (b) {
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B9600;
        }
    }

    bool openPort(const std::string& port, int baud) {
        fd = open(port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            int e = errno;
            if (e == EACCES) {
                setError("No permission for " + port + ". Add your user to the 'dialout' group (sudo usermod -aG dialout $USER) and log in again.");
            }
            else {
                setError("Cannot open " + port + ": " + strerror(e));
            }
            return false;
        }
        struct termios tio;
        if (tcgetattr(fd, &tio) == 0) {
            cfmakeraw(&tio);
            cfsetispeed(&tio, baudConst(baud));
            cfsetospeed(&tio, baudConst(baud));
            tio.c_cflag |= (CLOCAL | CREAD);
            tcsetattr(fd, TCSANOW, &tio); // ignored for ttyACM (USB CDC), harmless
        }
        return true;
    }

    int readSome(char* buf, int n) {
        struct pollfd p = { fd, POLLIN, 0 };
        int r = poll(&p, 1, 200);
        if (r <= 0) { return r; }
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) { return -1; }
        ssize_t got = read(fd, buf, n);
        if (got < 0 && (errno == EAGAIN || errno == EINTR)) { return 0; }
        return (int)got;
    }

    void closePort() {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }
#endif

    void run() {
        char buf[256];
        while (running) {
            int n = readSome(buf, sizeof(buf));
            if (n < 0) {
                setError("GPS device disconnected");
                running = false;
                break;
            }
            if (n > 0) { feed(buf, (size_t)n); }
        }
    }

    std::atomic<bool> running { false };
    std::thread worker;
    std::mutex mtx;
    GpsFix fix;
    std::string error;
    std::string line;
    std::chrono::steady_clock::time_point lastSentence;
    std::chrono::steady_clock::time_point lastValid;
};
