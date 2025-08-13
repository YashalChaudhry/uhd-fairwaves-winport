# uhd-fairwaves-winport
Side-by-side setup of UHD and UHD-Fairwaves for Windows, including build instructions and progress files.
Got it — here’s your manual rewritten as **professional GitHub-style Markdown** so you can drop it straight into a `README.md` file.

**Manual: Building UHD-Fairwaves on Windows**

**1. Prerequisites – Install Required Tools**

Before building **UHD-Fairwaves**, ensure the following tools are installed and configured.

**1.1 Python**

- Download Python 3.10.x (e.g., **3.10.10**) from:  
  🔗 [https://www.python.org/downloads/release/python-31010/](https://www.python.org/downloads/release/python-31010/)
- During installation:
  - **Check** “Add Python to PATH”.
  - Install for all users.

---

**1.2 Visual Studio 2022 Community Edition**

- Download from:  
  🔗 [https://visualstudio.microsoft.com/vs/community/](https://visualstudio.microsoft.com/vs/community/)
- During installation, **select**:
  - *Desktop development with C++* workload.
  - Ensure MSVC, Windows SDK, and CMake components are checked.

---

### **1.3 CMake**

- Download from:  
  🔗 [https://cmake.org/download/](https://cmake.org/download/)
- During installation:
  - Select “Add CMake to system PATH for all users”.

---

**1.4 Git**

- Download from:  
  🔗 [https://git-scm.com/download/win](https://git-scm.com/download/win)
- Add Git to PATH during installation.

---

**2. Install Dependencies via vcpkg**

We will use **vcpkg** to install Boost, pkgconf, and other UHD dependencies.

**2.1 Clone and Bootstrap vcpkg**

```powershell
cd %USERPROFILE%\Desktop
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
````

🔗 vcpkg repository: [https://github.com/microsoft/vcpkg](https://github.com/microsoft/vcpkg)

---

**2.2 Install Boost Libraries**

```powershell
.\vcpkg install boost-atomic:x64-windows boost-chrono:x64-windows boost-thread:x64-windows boost-system:x64-windows boost-date-time:x64-windows boost-program-options:x64-windows boost-random:x64-windows boost-serialization:x64-windows boost-test:x64-windows boost-regex:x64-windows boost-log:x64-windows boost-format:x64-windows boost-filesystem:x64-windows boost-graph:x64-windows
```

---

**2.3 Install pkgconf (pkg-config support)**

```powershell
.\vcpkg install pkgconf:x64-windows
```

---

**2.4 Integrate vcpkg with Visual Studio**

```powershell
.\vcpkg integrate install
```

This enables automatic detection of vcpkg-installed packages during CMake configuration.

---

**3. Build and Install Official UHD**

**3.1 Clone UHD**

```powershell
cd %USERPROFILE%\Desktop
git clone https://github.com/EttusResearch/uhd.git
cd uhd
```

🔗 UHD repository: [https://github.com/EttusResearch/uhd](https://github.com/EttusResearch/uhd)

---

**3.2 Create Build Directory**

```powershell
mkdir build
cd build
```

---
**3.3 Configure UHD (x64 Example)**

```powershell
cmake .. -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_TOOLCHAIN_FILE=C:/Users/Yashal/vcpkg/scripts/buildsystems/vcpkg.cmake
```

---

**3.4 Build UHD in Release Mode**

```powershell
cmake --build . --config Release
```

---

**3.5 Install UHD**

```powershell
cmake --install . --config Release
```

> **Note:** To install UHD to a custom directory, add:
> `-DCMAKE_INSTALL_PREFIX="C:/UHD_Custom"` to the CMake configure step.

---

**4. Build UHD-Fairwaves**

UHD-Fairwaves is a custom UHD variant maintained by Fairwaves.

**4.1 Clone UHD-Fairwaves**

```powershell
cd %USERPROFILE%\Desktop
git clone https://github.com/fairwaves/UHD-Fairwaves.git
cd UHD-Fairwaves/host
```

🔗 UHD-Fairwaves repository: [https://github.com/fairwaves/UHD-Fairwaves](https://github.com/fairwaves/UHD-Fairwaves)

---

**4.2 Create Build Directory**

```powershell
mkdir build
cd build
```

---

**4.3 Configure UHD-Fairwaves**

If UHD was installed to a custom directory:

```powershell
cmake -G "Visual Studio 17 2022" -A x64 .. ^
  -DCMAKE_PREFIX_PATH="C:/UHD_Custom"
```

If UHD was installed to the default CMake location, update the path accordingly.

---

**4.4 Build UHD-Fairwaves**

```powershell
cmake --build . --config Release
```

---

**5. Notes & Troubleshooting**
Ensure all dependencies are built for the same architecture (**x64**).
If `pkg-config` is not found, verify that `pkgconf` from vcpkg is installed and vcpkg integration is active.
Use `-A arm64` instead of `-A x64` if building for ARM64-based Windows.
Run all commands in Developer Command Prompt for VS 2022 to ensure MSVC environment variables are set.
