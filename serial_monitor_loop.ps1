$portName = "COM8"
$baudRate = 115200

# Set up the serial port object
$port = New-Object System.IO.Ports.SerialPort $portName, $baudRate, None, 8, one
$port.ReadTimeout = 200
$port.WriteTimeout = 200

try {
    $port.Open()
    Write-Host "=== Serial Monitor Opened on $portName ($baudRate Baud) ==="
    Write-Host "=== Resetting ESP32-S3 via DTR/RTS... ==="
    
    # Toggle RTS/DTR to reset ESP32-S3
    $port.DtrEnable = $false
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 200
    $port.DtrEnable = $false
    $port.RtsEnable = $false
    Start-Sleep -Milliseconds 200
    
    Write-Host "=== Monitoring started. Output streaming: ==="
    
    # Infinite loop reading data
    while ($true) {
        try {
            $data = $port.ReadExisting()
            if ($data.Length -gt 0) {
                Write-Host -NoNewline $data
            }
        } catch [TimeoutException] {
            # normal timeout, do nothing
        } catch {
            Write-Host "`nError reading from serial: $_"
            break
        }
        Start-Sleep -Milliseconds 20
    }
} catch {
    Write-Host "Failed to open serial port $portName. Make sure it is not in use by another application. Details: $_"
} finally {
    if ($port -and $port.IsOpen) {
        $port.Close()
        Write-Host "`n=== Serial Monitor Closed ==="
    }
}
