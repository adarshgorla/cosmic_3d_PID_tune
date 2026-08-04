$port = New-Object System.IO.Ports.SerialPort COM8, 115200, None, 8, one
$port.ReadTimeout = 1000
try {
    $port.Open()
    Write-Output "Serial port COM8 opened successfully."
    
    # Reset: EN = L (reset active), IO0 = H
    $port.DtrEnable = $false
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 200
    
    # Release: EN = H (run), IO0 = H
    $port.DtrEnable = $false
    $port.RtsEnable = $false
    Start-Sleep -Milliseconds 200
    
    Write-Output "ESP32-S3 reset toggled. Reading output for 15 seconds..."
    
    $start = Get-Date
    while (((Get-Date) - $start).TotalSeconds -lt 15) {
        try {
            $line = $port.ReadLine()
            Write-Output $line
        } catch {
            # Ignore timeout
        }
    }
} catch {
    Write-Error "Failed: $_"
} finally {
    if ($port.IsOpen) {
        $port.Close()
        Write-Output "Serial port COM8 closed."
    }
}
