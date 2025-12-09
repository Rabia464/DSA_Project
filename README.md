# DSA_Project

## Basic Browser in C++

A simple web browser application built with C++ and Qt6 WebEngine.

### Features

- **Web Navigation**: Browse websites using the address bar
- **Navigation Controls**: Back, forward, refresh, and home buttons
- **Smart URL Handling**: Automatically adds https:// for domains or converts text to Google search
- **Progress Indicator**: Visual progress bar while loading pages
- **Status Bar**: Shows loading status and page information

### Prerequisites

- **Qt6** (version 6.0 or higher) with the following components:
  - Qt6 Core
  - Qt6 WebEngineWidgets
  - Qt6 Widgets
- **CMake** (version 3.16 or higher)
- **C++17** compatible compiler (GCC, Clang, or MSVC)

### Installation

#### Windows

1. Download and install Qt6 from [qt.io](https://www.qt.io/download)
2. Make sure Qt6 is added to your PATH, or set `CMAKE_PREFIX_PATH` to your Qt6 installation directory
3. Install CMake from [cmake.org](https://cmake.org/download/)

#### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install qt6-base-dev qt6-webengine-dev cmake build-essential
```

#### macOS

```bash
brew install qt@6 cmake
```

### Building the Project

1. Create a build directory:
```bash
mkdir build
cd build
```

2. Configure with CMake:
```bash
cmake ..
```

   On Windows, you may need to specify the Qt6 path:
```bash
cmake .. -DCMAKE_PREFIX_PATH="C:/Qt/6.x.x/msvc2019_64"
```

3. Build the project:
```bash
cmake --build .
```

   Or use your system's build tool:
   - Windows: `cmake --build . --config Release`
   - Linux/macOS: `make`

4. Run the browser:
```bash
# Windows
.\bin\browser.exe

# Linux/macOS
./bin/browser
```

### Usage

1. **Navigate to a URL**: Type a URL in the address bar and press Enter or click "Go"
2. **Search**: Type a search query in the address bar - it will automatically search on Google
3. **Navigation**: Use the arrow buttons to go back/forward in your browsing history
4. **Refresh**: Click the refresh button (⟳) to reload the current page
5. **Home**: Click the home button (⌂) to go to the default homepage (Google)

### Project Structure

```
.
├── browser.cpp      # Main browser application source code
├── CMakeLists.txt   # CMake build configuration
└── README.md        # This file
```

### Notes

- The browser uses Qt WebEngine, which is based on Chromium, so it supports modern web standards
- First launch may take a moment as Qt WebEngine initializes
- Some websites may require additional permissions or settings

### Troubleshooting

**CMake can't find Qt6:**
- Set `CMAKE_PREFIX_PATH` to your Qt6 installation directory
- Make sure Qt6 is properly installed and the bin directory is in your PATH

**Build errors:**
- Ensure you have all required Qt6 components installed
- Check that your compiler supports C++17
- Verify CMake version is 3.16 or higher

**Runtime errors:**
- Make sure Qt6 WebEngine is properly installed
- On Linux, you may need to install additional dependencies: `sudo apt-get install libqt6webenginewidgets6`

