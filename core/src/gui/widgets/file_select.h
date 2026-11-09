#pragma once
#include <imgui.h>
#include <imgui_internal.h>
#include <stdint.h>
#include <string>
#include <thread>
#include <vector>

class FileSelect {
public:
    FileSelect(std::string defaultPath, std::vector<std::string> filter = { "All Files", "*" });
    bool render(std::string id);
    void setPath(std::string path, bool markChanged = false);
    bool pathIsValid();
    // AIA: open the file dialog without drawing the field (e.g. from a softkey)
    // and poll the result in frames where render() is not called
    void openDialog();
    bool pollChanged();

    std::string expandString(std::string input);

    std::string path = "";
    bool dialogOpen = false;

private:
    void worker();
    std::thread workerThread;
    std::vector<std::string> _filter;
    std::string root = "";

    bool pathValid = false;
    char strPath[2048];
    bool pathChanged = false;
};