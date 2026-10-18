#pragma once
// Opens a file or folder with the desktop's default application (browser for
// HTML, file manager for folders) and reports whether that worked.
//
// Linux: runs xdg-open (then gio, kde-open5, kde-open) in its own session with
// a clean environment - without the AppImage's library path, so the browser
// and the desktop helpers use the system libraries. The result (exit code and
// the helper's messages) is shown in the GUI. Windows: ShellExecute.

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <utils/flog.h>

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#pragma comment(lib, "shell32.lib")
#else
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
extern char** environ;
#endif

class ExternalOpener {
public:
    enum State { IDLE, RUNNING, OK, FAILED };

    ~ExternalOpener() {
        if (worker.joinable()) { worker.join(); }
    }

    // starts opening 'path' (asynchronously on Linux); ignored while one is running
    void open(const std::string& path) {
        if (state.load() == RUNNING) { return; }
        if (worker.joinable()) { worker.join(); }
        {
            std::lock_guard<std::mutex> lck(mtx);
            msg.clear();
            target = path;
        }
#ifdef _WIN32
        HINSTANCE r = ShellExecuteA(NULL, "open", path.c_str(), NULL, NULL, SW_SHOWNORMAL);
        if ((INT_PTR)r > 32) { state.store(OK); }
        else {
            std::lock_guard<std::mutex> lck(mtx);
            msg = "Windows could not open it (error " + std::to_string((long long)(INT_PTR)r) + ").";
            state.store(FAILED);
        }
#else
        state.store(RUNNING);
        worker = std::thread(&ExternalOpener::run, this, path);
#endif
    }

    State getState() const { return state.load(); }
    std::string message() {
        std::lock_guard<std::mutex> lck(mtx);
        return msg;
    }
    std::string lastTarget() {
        std::lock_guard<std::mutex> lck(mtx);
        return target;
    }

#ifndef _WIN32
    // environment for the helper: everything except what the AppImage
    // launcher sets for its own libraries
    static std::vector<std::string> cleanEnvironment() {
        static const char* drop[] = { "LD_LIBRARY_PATH=", "LD_PRELOAD=", "SDRPP_APPDIR=", "APPDIR=", "APPIMAGE=", "ARGV0=", "OWD=",
                                      "QT_PLUGIN_PATH=", "QT_QPA_PLATFORM_PLUGIN_PATH=", "GTK_PATH=", "GIO_MODULE_DIR=",
                                      "GDK_PIXBUF_MODULE_FILE=", "PYTHONHOME=", "PYTHONPATH=", "PERLLIB=", "GSETTINGS_SCHEMA_DIR=" };
        std::vector<std::string> env;
        for (char** e = environ; e && *e; e++) {
            bool skip = false;
            for (const char* d : drop) {
                if (strncmp(*e, d, strlen(d)) == 0) { skip = true; break; }
            }
            if (!skip) { env.push_back(*e); }
        }
        return env;
    }
#endif

private:
#ifndef _WIN32
    void run(std::string path) {
        // everything the child needs is prepared before fork (multithreaded process)
        std::vector<std::string> envs = cleanEnvironment();
        std::vector<char*> envp;
        for (auto& e : envs) { envp.push_back((char*)e.c_str()); }
        envp.push_back(nullptr);
        static const char* script =
            "f=\"$1\"; "
            "if command -v xdg-open >/dev/null 2>&1; then xdg-open \"$f\" && exit 0; fi; "
            "if command -v gio >/dev/null 2>&1; then gio open \"$f\" && exit 0; fi; "
            "if command -v kde-open5 >/dev/null 2>&1; then kde-open5 \"$f\" && exit 0; fi; "
            "if command -v kde-open >/dev/null 2>&1; then kde-open \"$f\" && exit 0; fi; "
            "echo 'no opener (xdg-open, gio, kde-open) succeeded' >&2; exit 3";
        const char* argv[] = { "/bin/sh", "-c", script, "sh", path.c_str(), nullptr };

        int pfd[2];
        if (pipe(pfd) != 0) { finish(false, "Could not start the opener (pipe)."); return; }
        pid_t pid = fork();
        if (pid < 0) {
            close(pfd[0]); close(pfd[1]);
            finish(false, "Could not start the opener (fork).");
            return;
        }
        if (pid == 0) {
            setsid();
            dup2(pfd[1], 1);
            dup2(pfd[1], 2);
            close(pfd[0]);
            close(pfd[1]);
            int dn = ::open("/dev/null", O_RDONLY);
            if (dn >= 0) { dup2(dn, 0); close(dn); }
            execve("/bin/sh", (char* const*)argv, envp.data());
            _exit(127);
        }
        close(pfd[1]);
        // read the helper's messages; xdg-open normally returns as soon as the
        // application is started. After 15 s still running: assume success.
        std::string out;
        int status = 0;
        bool exited = false;
        struct timespec t0;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        while (true) {
            struct pollfd p = { pfd[0], POLLIN, 0 };
            int pr = poll(&p, 1, 200);
            if (pr > 0) {
                char buf[256];
                ssize_t n = read(pfd[0], buf, sizeof(buf));
                if (n > 0 && out.size() < 2000) { out.append(buf, (size_t)n); }
            }
            pid_t w = waitpid(pid, &status, WNOHANG);
            if (w == pid) { exited = true; break; }
            if (w < 0 && errno == ECHILD) { break; }   // reaped elsewhere (SIGCHLD ignored): result unknown
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            if ((t1.tv_sec - t0.tv_sec) > 15) { break; }
        }
        close(pfd[0]);
        if (!exited) {
            // still running (some openers wait for the application): reap later
            std::thread([pid]() { int st; waitpid(pid, &st, 0); }).detach();
            finish(true, "");
            return;
        }
        bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) { out.pop_back(); }
        if (!ok) {
            std::string m = "The desktop could not open it";
            if (WIFEXITED(status)) { m += " (exit " + std::to_string(WEXITSTATUS(status)) + ")"; }
            if (!out.empty()) { m += ": " + (out.size() > 400 ? out.substr(0, 400) + " ..." : out); }
            flog::warn("TETRA: opening '{}' failed: {}", path, m);
            finish(false, m);
        }
        else { finish(true, ""); }
    }

    void finish(bool ok, const std::string& m) {
        {
            std::lock_guard<std::mutex> lck(mtx);
            msg = m;
        }
        state.store(ok ? OK : FAILED);
    }
#endif

    std::atomic<State> state { IDLE };
    std::mutex mtx;
    std::string msg, target;
    std::thread worker;
};

// Default folder for logs, maps and reports. Linux: when the config folder is
// hidden (AppImage: ~/.config/...), use the user's documents folder instead -
// browsers installed as snap cannot open files in hidden folders.
inline std::string tetraDefaultLogDir(const std::string& expandedRoot) {
#if defined(_WIN32) || defined(__APPLE__)
    (void)expandedRoot;
    return "%ROOT%/tetra_logs";
#else
    if (expandedRoot.find("/.") == std::string::npos) { return "%ROOT%/tetra_logs"; }
    const char* home = getenv("HOME");
    if (!home || !*home) { return "%ROOT%/tetra_logs"; }
    std::string h = home;
    std::string docs;
    // XDG user dirs: XDG_DOCUMENTS_DIR="$HOME/Dokumente"
    std::string ud = h + "/.config/user-dirs.dirs";
    if (const char* xc = getenv("XDG_CONFIG_HOME")) { if (*xc) { ud = std::string(xc) + "/user-dirs.dirs"; } }
    if (FILE* f = fopen(ud.c_str(), "r")) {
        char line[512];
        while (fgets(line, sizeof(line), f)) {
            if (strncmp(line, "XDG_DOCUMENTS_DIR=", 18) != 0) { continue; }
            std::string v = line + 18;
            while (!v.empty() && (v.back() == '\n' || v.back() == '\r' || v.back() == '"')) { v.pop_back(); }
            if (!v.empty() && v[0] == '"') { v.erase(0, 1); }
            if (v.rfind("$HOME", 0) == 0) { v = h + v.substr(5); }
            if (!v.empty() && v[0] == '/' && v != h && v != h + "/") { docs = v; }
        }
        fclose(f);
    }
    if (docs.empty()) {
        struct stat st;
        std::string d = h + "/Documents";
        docs = (stat(d.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) ? d : h;
    }
    return docs + "/SDRPP-TETRA-SCHWARTZ";
#endif
}
