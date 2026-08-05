[CmdletBinding()]
param(
    [ValidateSet('Probe', 'Run')]
    [string]$Action = 'Probe',
    [string]$Port = 'COM11',
    [int]$BaudRate = 115200,
    [double]$LeftKp = 2.0,
    [double]$LeftKi = 0.0,
    [double]$RightKp = 2.0,
    [double]$RightKi = 0.0,
    [int]$LeftTarget = 80,
    [int]$RightTarget = 80,
    [int]$LeftMaxPwm = 2000,
    [int]$RightMaxPwm = 2000,
    [ValidateRange(100, 1500)]
    [int]$DurationMs = 300,
    [ValidateRange(500, 25000)]
    [int]$PulseLimit = 3000,
    [string]$LogRoot = (Join-Path $PSScriptRoot 'speed_tune_logs')
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:Sequence = 1
$script:PortHandle = $null
$script:RunWasRequested = $false

function Get-NextSequence {
    $value = $script:Sequence
    $script:Sequence++
    if ($script:Sequence -gt 65535) {
        $script:Sequence = 1
    }
    return $value
}

function New-SpeedTuneFrame {
    param([Parameter(Mandatory)][string]$Payload)

    [byte]$checksum = 0
    foreach ($value in [System.Text.Encoding]::ASCII.GetBytes($Payload)) {
        $checksum = $checksum -bxor $value
    }
    return ('{0}*{1:X2}' -f $Payload, $checksum)
}

function Send-SpeedTunePayload {
    param([Parameter(Mandatory)][string]$Payload)

    $frame = New-SpeedTuneFrame -Payload $Payload
    $script:PortHandle.WriteLine($frame)
}

function Read-SpeedTuneLine {
    try {
        $line = $script:PortHandle.ReadLine()
        if ($null -eq $line) {
            return $null
        }
        return $line.Trim()
    }
    catch [System.TimeoutException] {
        return $null
    }
}

function Wait-SpeedTuneResponse {
    param(
        [Parameter(Mandatory)][int]$Sequence,
        [Parameter(Mandatory)][string]$Prefix,
        [int]$TimeoutMs = 2000
    )

    $deadline = [DateTime]::UtcNow.AddMilliseconds($TimeoutMs)
    while ([DateTime]::UtcNow -lt $deadline) {
        $line = Read-SpeedTuneLine
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        if ($line.StartsWith("ERR,$Sequence,") -or $line.StartsWith('ERR,0,')) {
            throw "Device rejected command: $line"
        }
        if ($line.StartsWith("$Prefix,$Sequence,")) {
            return $line
        }
    }
    throw "Timed out waiting for $Prefix response to sequence $Sequence."
}

function Invoke-SpeedTuneCommand {
    param(
        [Parameter(Mandatory)][string]$Command,
        [string[]]$Arguments = @(),
        [string]$ResponsePrefix = 'ACK',
        [int]$TimeoutMs = 2000
    )

    $sequence = Get-NextSequence
    $parts = @($Command, $sequence) + $Arguments
    Send-SpeedTunePayload -Payload ($parts -join ',')
    return Wait-SpeedTuneResponse -Sequence $sequence -Prefix $ResponsePrefix -TimeoutMs $TimeoutMs
}

function Get-SpeedTuneState {
    $response = Invoke-SpeedTuneCommand -Command 'GET' -ResponsePrefix 'STATE'
    $parts = $response -split ','
    if ($parts.Count -lt 15) {
        throw "Malformed STATE response: $response"
    }
    return [pscustomobject]@{
        Configured = [int]$parts[2]
        Armed = [int]$parts[3]
        Running = [int]$parts[4]
        Ch5Low = [int]$parts[5]
        Ch5High = [int]$parts[6]
        LeftKpMilli = [int]$parts[7]
        LeftKiMilli = [int]$parts[8]
        RightKpMilli = [int]$parts[9]
        RightKiMilli = [int]$parts[10]
        LeftTarget = [int]$parts[11]
        RightTarget = [int]$parts[12]
        LeftMaxPwm = [int]$parts[13]
        RightMaxPwm = [int]$parts[14]
    }
}

function Get-Mean {
    param([double[]]$Values)
    if ($Values.Count -eq 0) { return 0.0 }
    return ($Values | Measure-Object -Average).Average
}

function Get-StandardDeviation {
    param([double[]]$Values)
    if ($Values.Count -eq 0) { return 0.0 }
    $mean = Get-Mean -Values $Values
    $sum = 0.0
    foreach ($value in $Values) {
        $delta = $value - $mean
        $sum += $delta * $delta
    }
    return [Math]::Sqrt($sum / $Values.Count)
}

function Get-SignSwitchCount {
    param([int[]]$Values)
    $count = 0
    $lastSign = 0
    foreach ($value in $Values) {
        $sign = [Math]::Sign($value)
        if ($sign -ne 0) {
            if ($lastSign -ne 0 -and $sign -ne $lastSign) {
                $count++
            }
            $lastSign = $sign
        }
    }
    return $count
}

function Save-SpeedTuneResult {
    param(
        [Parameter(Mandatory)][System.Collections.Generic.List[object]]$Samples,
        [Parameter(Mandatory)][string]$DoneLine
    )

    New-Item -ItemType Directory -Force -Path $LogRoot | Out-Null
    $stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
    $baseName = "run_${stamp}_T${LeftTarget}_Kp${LeftKp}_Ki${LeftKi}"
    $csvPath = Join-Path $LogRoot "$baseName.csv"
    $summaryPath = Join-Path $LogRoot "$baseName.summary.json"
    $Samples | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8

    [double[]]$leftActual = @($Samples | ForEach-Object { [double]$_.LeftActual })
    [double[]]$rightActual = @($Samples | ForEach-Object { [double]$_.RightActual })
    [int[]]$leftOut = @($Samples | ForEach-Object { [int]$_.LeftOut })
    [int[]]$rightOut = @($Samples | ForEach-Object { [int]$_.RightOut })
    $leftMean = Get-Mean -Values $leftActual
    $rightMean = Get-Mean -Values $rightActual
    $summary = [ordered]@{
        Done = $DoneLine
        Samples = $Samples.Count
        DurationMsFromSamples = $Samples.Count * 10
        LeftTarget = $LeftTarget
        RightTarget = $RightTarget
        LeftKp = $LeftKp
        LeftKi = $LeftKi
        RightKp = $RightKp
        RightKi = $RightKi
        LeftMaxPwm = $LeftMaxPwm
        RightMaxPwm = $RightMaxPwm
        LeftActualMean = [Math]::Round($leftMean, 3)
        RightActualMean = [Math]::Round($rightMean, 3)
        LeftMeanError = [Math]::Round($LeftTarget - $leftMean, 3)
        RightMeanError = [Math]::Round($RightTarget - $rightMean, 3)
        LeftActualStd = [Math]::Round((Get-StandardDeviation -Values $leftActual), 3)
        RightActualStd = [Math]::Round((Get-StandardDeviation -Values $rightActual), 3)
        LeftActualMin = if ($Samples.Count) { ($leftActual | Measure-Object -Minimum).Minimum } else { 0 }
        LeftActualMax = if ($Samples.Count) { ($leftActual | Measure-Object -Maximum).Maximum } else { 0 }
        RightActualMin = if ($Samples.Count) { ($rightActual | Measure-Object -Minimum).Minimum } else { 0 }
        RightActualMax = if ($Samples.Count) { ($rightActual | Measure-Object -Maximum).Maximum } else { 0 }
        LeftSaturationCount = @($leftOut | Where-Object { [Math]::Abs($_) -ge $LeftMaxPwm }).Count
        RightSaturationCount = @($rightOut | Where-Object { [Math]::Abs($_) -ge $RightMaxPwm }).Count
        LeftOutputSignSwitches = Get-SignSwitchCount -Values $leftOut
        RightOutputSignSwitches = Get-SignSwitchCount -Values $rightOut
    }
    $summary | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $summaryPath -Encoding UTF8
    Write-Host "CSV=$csvPath"
    Write-Host "SUMMARY=$summaryPath"
    Write-Host ($summary | ConvertTo-Json -Compress)
}

try {
    $script:PortHandle = [System.IO.Ports.SerialPort]::new(
        $Port,
        $BaudRate,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One)
    $script:PortHandle.NewLine = "`n"
    $script:PortHandle.Encoding = [System.Text.Encoding]::ASCII
    $script:PortHandle.ReadTimeout = 50
    $script:PortHandle.WriteTimeout = 500
    $script:PortHandle.DtrEnable = $false
    $script:PortHandle.RtsEnable = $false
    $script:PortHandle.Open()
    Start-Sleep -Milliseconds 250
    $script:PortHandle.DiscardInBuffer()
    $script:PortHandle.DiscardOutBuffer()

    $hello = Invoke-SpeedTuneCommand -Command 'HELLO' -ResponsePrefix 'HELLO'
    Write-Host $hello

    if ($Action -eq 'Probe') {
        $state = Get-SpeedTuneState
        Write-Host ($state | ConvertTo-Json -Compress)
        return
    }

    $gainValues = @(
        [int][Math]::Round($LeftKp * 1000.0),
        [int][Math]::Round($LeftKi * 1000.0),
        [int][Math]::Round($RightKp * 1000.0),
        [int][Math]::Round($RightKi * 1000.0),
        $LeftTarget,
        $RightTarget,
        $LeftMaxPwm,
        $RightMaxPwm) | ForEach-Object { $_.ToString() }
    Write-Host (Invoke-SpeedTuneCommand -Command 'CFG' -Arguments $gainValues)

    $state = Get-SpeedTuneState
    if (($state.LeftKpMilli -ne [int]$gainValues[0]) -or
        ($state.LeftKiMilli -ne [int]$gainValues[1]) -or
        ($state.RightKpMilli -ne [int]$gainValues[2]) -or
        ($state.RightKiMilli -ne [int]$gainValues[3]) -or
        ($state.LeftTarget -ne $LeftTarget) -or
        ($state.RightTarget -ne $RightTarget) -or
        ($state.LeftMaxPwm -ne $LeftMaxPwm) -or
        ($state.RightMaxPwm -ne $RightMaxPwm)) {
        throw 'Device CFG readback does not match the requested values.'
    }
    if ($state.Ch5Low -ne 1) {
        throw 'CH5 must be LOW and the receiver must be online before ARM.'
    }
    Write-Host (Invoke-SpeedTuneCommand -Command 'ARM')
    $script:RunWasRequested = $true
    Write-Host 'ARMED_WAIT_CH5_HIGH'

    $highDeadline = [DateTime]::UtcNow.AddSeconds(25)
    do {
        Start-Sleep -Milliseconds 200
        $state = Get-SpeedTuneState
        if ($state.Ch5High -eq 1) {
            break
        }
    } while ([DateTime]::UtcNow -lt $highDeadline)
    if ($state.Ch5High -ne 1) {
        throw 'Timed out waiting for CH5 HIGH.'
    }

    $runSequence = Get-NextSequence
    Send-SpeedTunePayload -Payload ("RUN,$runSequence,$DurationMs,$PulseLimit")
    Write-Host "RUN_SENT,$runSequence"

    $samples = [System.Collections.Generic.List[object]]::new()
    $doneLine = $null
    $runAcknowledged = $false
    $deadline = [DateTime]::UtcNow.AddMilliseconds($DurationMs + 2500)
    $nextHeartbeat = [DateTime]::UtcNow

    while ([DateTime]::UtcNow -lt $deadline -and $null -eq $doneLine) {
        if ([DateTime]::UtcNow -ge $nextHeartbeat) {
            $heartbeatSequence = Get-NextSequence
            Send-SpeedTunePayload -Payload ("HB,$heartbeatSequence")
            $nextHeartbeat = [DateTime]::UtcNow.AddMilliseconds(100)
        }

        $line = Read-SpeedTuneLine
        if ([string]::IsNullOrWhiteSpace($line)) {
            continue
        }
        if ($line -eq "ACK,$runSequence,RUN") {
            $runAcknowledged = $true
            continue
        }
        if ($line.StartsWith("ERR,$runSequence,")) {
            throw "RUN rejected: $line"
        }
        if ($line.StartsWith('D,')) {
            $parts = $line -split ','
            if ($parts.Count -eq 13) {
                $samples.Add([pscustomobject]@{
                    RunId = [int]$parts[1]
                    Tick = [int]$parts[2]
                    LeftTarget = [int]$parts[3]
                    LeftActual = [int]$parts[4]
                    LeftOut = [int]$parts[5]
                    LeftError = [int]$parts[6]
                    LeftIOut = [int]$parts[7]
                    RightTarget = [int]$parts[8]
                    RightActual = [int]$parts[9]
                    RightOut = [int]$parts[10]
                    RightError = [int]$parts[11]
                    RightIOut = [int]$parts[12]
                })
            }
            continue
        }
        if ($line.StartsWith("DONE,$runSequence,")) {
            $doneLine = $line
        }
    }

    if (-not $runAcknowledged) {
        throw 'RUN was not acknowledged.'
    }
    if ($null -eq $doneLine) {
        throw 'No DONE event received before host timeout.'
    }

    Write-Host $doneLine
    Write-Host 'RUN_FINISHED_SET_CH5_LOW'
    Save-SpeedTuneResult -Samples $samples -DoneLine $doneLine
}
finally {
    if ($null -ne $script:PortHandle) {
        try {
            if ($script:PortHandle.IsOpen -and $script:RunWasRequested) {
                1..3 | ForEach-Object {
                    $stopSequence = Get-NextSequence
                    Send-SpeedTunePayload -Payload ("STOP,$stopSequence")
                    Start-Sleep -Milliseconds 30
                }
            }
        }
        catch {
            Write-Warning "Unable to send final STOP: $($_.Exception.Message)"
        }
        if ($script:PortHandle.IsOpen) {
            $script:PortHandle.Close()
        }
        $script:PortHandle.Dispose()
    }
}
