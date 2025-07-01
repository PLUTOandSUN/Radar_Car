# STM32 Project Build Script
Write-Host "Building STM32 Project..." -ForegroundColor Green

# Set environment
$env:PATH += ";D:\Projects\arm-none-eabi\14.2 rel1\bin"

# Check if we need to reconfigure CMake
$needReconfigure = $false
if (Test-Path "build/Debug/CMakeCache.txt") {
    $cacheContent = Get-Content "build/Debug/CMakeCache.txt" -Raw
    if ($cacheContent -match "stm32cube/bundles" -or $cacheContent -notmatch "D:/Projects/arm-none-eabi") {
        Write-Host "Detected old compiler path in cache, will reconfigure..." -ForegroundColor Yellow
        $needReconfigure = $true
    }
}

if ($needReconfigure -or !(Test-Path "build/Debug/build.ninja")) {
    Write-Host "Cleaning and reconfiguring CMake..." -ForegroundColor Yellow
    if (Test-Path "build") {
        Remove-Item -Recurse -Force "build"
    }
    cmake --preset Debug
    if ($LASTEXITCODE -ne 0) {
        Write-Host "CMake configuration failed!" -ForegroundColor Red
        exit 1
    }
}

# Build project
Write-Host "Compiling..." -ForegroundColor Yellow
cmake --build --preset Debug

if ($LASTEXITCODE -eq 0) {
    Write-Host "Build successful!" -ForegroundColor Green
    
    # Generate HEX file
    Write-Host "Generating HEX file..." -ForegroundColor Yellow
    arm-none-eabi-objcopy -O ihex build/Debug/Radar_Car.elf build/Debug/Radar_Car.hex
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "HEX file generated successfully!" -ForegroundColor Green
        Write-Host ""
        Write-Host "Generated files:" -ForegroundColor Cyan
        Write-Host "  - build/Debug/Radar_Car.elf  (debug file)" -ForegroundColor White
        Write-Host "  - build/Debug/Radar_Car.hex  (flash file)" -ForegroundColor White
        Write-Host ""
        Write-Host "To flash firmware:" -ForegroundColor Magenta
        Write-Host "  STM32_Programmer_CLI.exe -c port=SWD -w build/Debug/Radar_Car.hex -v -rst" -ForegroundColor White
    } else {
        Write-Host "HEX file generation failed!" -ForegroundColor Red
    }
} else {
    Write-Host "Build failed! Please check code errors." -ForegroundColor Red
}