# Minimal interactive TCP console for the ESP32 motor test.
# Usage:
#   powershell -ExecutionPolicy Bypass -File wifi_console.ps1 -Ip 192.168.1.42
#
# Type l / L / r / R / b / B / s / e and press Enter.
# Press Ctrl+C to quit.
param(
    [Parameter(Mandatory = $true)][string]$Ip,
    [int]$Port = 23
)

$ErrorActionPreference = "Stop"

Write-Host "Connecting to $Ip`:$Port ..." -ForegroundColor Cyan
$client = New-Object System.Net.Sockets.TcpClient($Ip, $Port)
$stream = $client.GetStream()

Write-Host "Connected. Type your command letter and press Enter. Ctrl+C to quit." -ForegroundColor Green

$buffer = New-Object byte[] 4096

while ($client.Connected) {
    if ($stream.DataAvailable) {
        $n = $stream.Read($buffer, 0, $buffer.Length)
        if ($n -le 0) { break }
        Write-Host -NoNewline ([Text.Encoding]::ASCII.GetString($buffer, 0, $n))
    }
    if ([Console]::KeyAvailable) {
        $key = [Console]::ReadKey($true)
        if ($key.Key -eq [ConsoleKey]::Enter) {
            $bytes = [Text.Encoding]::ASCII.GetBytes("`r`n")
            $stream.Write($bytes, 0, $bytes.Length)
            Write-Host
        } elseif ($key.KeyChar) {
            $bytes = [Text.Encoding]::ASCII.GetBytes($key.KeyChar)
            $stream.Write($bytes, 0, $bytes.Length)
            Write-Host -NoNewline $key.KeyChar
        }
        $stream.Flush()
    }
    Start-Sleep -Milliseconds 30
}

Write-Host "`nConnection closed." -ForegroundColor Yellow
$client.Close()
